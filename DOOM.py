from typing import Optional, Union, List, Tuple
import time
from collections import namedtuple

import numpy as np
import gym
import vizdoom
from vizdoom import DoomGame, Button, GameVariable, ScreenFormat, ScreenResolution, AutomapMode, Mode


# TODO minimize sliding:
#   https://forum.zdoom.org/viewtopic.php?p=933227&sid=fdef5121aec04509a2b76c8048ea5cc5#p933227

# TODO make feedback() used in omnidrive a class object that stores the game setpoint and ways to call for current pos

# TODO LATER: make DOOM a subclass of a abstract class (GAME) to generalize over games
from gym.core import RenderFrame, ActType, ObsType


class DOOM(gym.Env):

    DEFAULT_CFG = {
        'mode': Mode.PLAYER,
        'res': ScreenResolution.RES_1920X1080,
        'win_visible': True,
        'fullscreen': True,
        'render_hud': False,
        'automap': False,
        'buttons': [Button.MOVE_FORWARD_BACKWARD_DELTA, Button.MOVE_LEFT_RIGHT_DELTA, Button.TURN_LEFT_RIGHT_DELTA,
                    Button.ATTACK],
        'ep_timeout': 1000 * 45,  # in ticks
        'skiprate': 4,
        'living_r': -.1 / 4. / 2.,
        'shot_r': -10,
        'dmg_taken_r': -1,  # / hp
        'max_player_speed': 500,  # cm/s; rat max is 500
    }

    # variable naming: https://github.com/mwydmuch/ViZDoom/issues/210
    #
    GAME_VARS = [
        GameVariable.POSITION_X, GameVariable.POSITION_Y, GameVariable.ANGLE,
        GameVariable.VELOCITY_X, GameVariable.VELOCITY_Y,
        GameVariable.HEALTH, GameVariable.DAMAGE_TAKEN, GameVariable.DEAD,
        GameVariable.HITCOUNT, GameVariable.KILLCOUNT, GameVariable.FRAGCOUNT, GameVariable.ATTACK_READY,
        GameVariable.ITEMCOUNT, GameVariable.USER1, GameVariable.USER2, GameVariable.USER3, GameVariable.USER4,
        GameVariable.USER5, GameVariable.USER6, GameVariable.USER7,
        GameVariable.AMMO0, GameVariable.AMMO1, GameVariable.AMMO2, GameVariable.AMMO3, GameVariable.AMMO4,
        GameVariable.AMMO5, GameVariable.AMMO6, GameVariable.AMMO7, GameVariable.AMMO8, GameVariable.AMMO9,
        GameVariable.SELECTED_WEAPON_AMMO,
    ]
    GAME_VAR_RENAMING = {GameVariable.USER1: 'kill_count', GameVariable.USER2: 'monsters_present',
                         GameVariable.USER3: 'monster_pos_x', GameVariable.USER4: 'monster_pos_y',
                         GameVariable.USER5: 'monsters_spawned', GameVariable.USER6: 'monsters_removed',
                         GameVariable.USER7: 'wall_bump_angle'}

    # named tuple, with names derived by lower casing game var names
    # use GAME_VAR_RENAMING if possible
    # GAME_STATE_T = namedtuple('game_state_t', [GAME_VAR_RENAMING.get(v, str(v)[str(v).find('.') + 1:].lower())
    #                                            for v in GAME_VARS])  # TODO can't do this here in py3

    def __init__(self, wad_path, map_id, cfg_update=None):
        super().__init__()

        self.game_state_t = namedtuple('game_state_t', [DOOM.GAME_VAR_RENAMING.get(v, str(v)[str(v).find('.') + 1:].lower())
                                                        for v in DOOM.GAME_VARS])

        self.cfg = DOOM.DEFAULT_CFG
        if cfg_update is not None:  # if cfg is given..
            self.cfg.update(cfg_update)  # update only the default

        self.game = DoomGame()
        self.game.set_doom_scenario_path(wad_path)
        self.game.set_doom_map(map_id)

        self.game.set_screen_resolution(self.cfg['res'])
        self.game.set_screen_format(ScreenFormat.RGB24)
        self.game.set_render_hud(self.cfg['render_hud'])
        # self.game.add_game_args("-width 3440 -height 1440")  # -record recordings
        self.game.add_game_args(f'+fullscreen 1 +viz_nocheat 0 +fullscreen {[0, 1][self.cfg["fullscreen"]]} +freelook 1')
        # self.game.add_game_args('+snd_mididevice -1 +snd_midipatchset /media/viktor/OS/csiki/rats_play_doom/doom/gm.dls')
        self.game.set_sound_enabled(True)  # TODO crashes because of old 1.19 openal version that sticks to vizdoom somewhow

        self.game.set_render_crosshair(False)
        self.game.set_render_weapon(True)
        self.game.set_render_decals(True)
        self.game.set_render_particles(True)

        self.game.set_automap_buffer_enabled(self.cfg['automap'])
        self.game.set_automap_rotate(False)
        self.game.set_automap_render_textures(False)
        self.game.set_automap_mode(AutomapMode.OBJECTS_WITH_SIZE)

        self.game.set_mode(self.cfg['mode'])

        buttons = set(self.cfg['buttons'])
        if self.cfg['mode'] == Mode.SPECTATOR:  # add btns for keyboard input to work
            buttons.update([Button.MOVE_FORWARD, Button.MOVE_BACKWARD, Button.MOVE_RIGHT, Button.MOVE_LEFT,
                            Button.TURN_LEFT, Button.TURN_RIGHT, Button.STRAFE])

        for b in buttons:
            self.game.add_available_button(b)

        for v in DOOM.GAME_VARS:
            self.game.add_available_game_variable(v)

        self.game.set_episode_timeout(self.cfg['ep_timeout'])
        self.game.set_episode_start_time(10)
        self.game.set_window_visible(self.cfg['win_visible'])

        self.game.set_living_reward(self.cfg['living_r'])
        self.game.set_console_enabled(True)

        self.game.init()

        # after-init settings
        self.game.set_console_enabled(True)
        if self.cfg['automap']:
            self.game.send_game_command('am_showmonsters true')
            self.game.send_game_command('am_colorset 2')

        # extra command settings
        self.game.send_game_command('sv_cheats 1')
        # self.game.send_game_command('fov 120')  # has to be called below for some reason

        # movement: https://www.reddit.com/r/Doom/comments/4nt3fo/i_got_bored_and_did_some_math_on_the_original/
        #   8 map units = 1 foot ~= 30 cm
        #   35 tics = 1 sec
        #   max speed originally is 30 mu/tic
        self.tic_per_sec = 35.
        self.map_unit_per_cm = 8. / 30.48
        self.map_degree_per_rad = 50 / np.pi  # TODO possibly need to implement in-game calibration too
        move_speed_rng = self.cfg['max_player_speed'] #/ self.tic_per_sec * self.map_unit_per_cm  # map_unit / tic
        move_turn_rng = (-0.5, 0.5)  # game degree in [0, 1]
        move_space = gym.spaces.Box(low=np.array([-move_speed_rng, -move_speed_rng, move_turn_rng[0]]),
                                    high=np.array([move_speed_rng, move_speed_rng, move_turn_rng[1]]))
        shoot_space = gym.spaces.Discrete(2)  # shoot or no
        self.action_space = gym.spaces.Tuple([move_space, shoot_space])
        self.observation_space = gym.spaces.Discrete(2)  # not really used, just here for gym's sake for now

        self.step_i = None
        self.start_ammo = None

    def _get_state(self):
        state = self.game.get_state()
        game_over = state is None
        game_state = None

        if not game_over:
            self.step_i = state.number
            game_vars = state.game_variables
            game_state = self.game_state_t(*game_vars)  # cast to namedtuple

        return game_state, game_over

    def step(self, action: ActType) -> Tuple[ObsType, float, bool, bool, dict]:
        step_start = time.time()

        # action
        if self.cfg['mode'] == Mode.PLAYER:
            move, shoot = np.array(action[0]), action[1]  # cpy move before altered
            move[:2] = move[:2] / self.map_unit_per_cm  # / self.tic_per_sec * self.map_unit_per_cm  # TODO !this made it too slow! todo * self.cfg['skiprate'] ?
            move[2] = move[2] * self.map_degree_per_rad  # TODO !this made it too slow!
            action = move.tolist() + [shoot]
            reward = self.game.make_action(action, self.cfg['skiprate'])
        elif self.cfg['mode'] == Mode.SPECTATOR:
            self.game.advance_action()
            action = self.game.get_last_action()
            reward = self.game.get_last_reward()
        else:
            raise NotImplemented('Async modes are not implemented!')

        # get state
        state, game_over = self._get_state()

        b_angle = 0
        b_distance = np.inf  # by default, infinite distance from walls
        if state.wall_bump_angle != -1:
            p_angle = state.angle
            w_angle = state.wall_bump_angle
            b_angle = (p_angle - w_angle) % 360  # bump angle, front is 0, increasing clockwise
            b_angle = -(360 - b_angle) if b_angle > 180 else b_angle  # [0,360] -> [-180,+180]
            b_distance = 0.  # only detecting bumps for now, air puffs are either on when hitting a wall or just off

        step_over = time.time()

        # TODO should save the state of the acs script: have inverse gitignore in doom/scenarios, or just in the root

        finished = self.game.is_episode_finished()
        return state, reward, state.dead, game_over or finished and not state.dead, \
               {'i': self.step_i, 'step_t': (step_over - step_start) * 1000,
                'bump_angle': b_angle, 'bump_distance': b_distance, 'action': action}

    def render(self) -> Optional[Union[RenderFrame, List[RenderFrame]]]:
        pass  # happens anyway

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None) -> Tuple[ObsType, dict]:
        # launch new episode
        self.game.new_episode()

        # send commands after episodes launches
        self.game.make_action([0, 0, 0, 0], self.cfg['skiprate'])
        self.game.send_game_command('fov 120')
        self.game.send_game_command('vid_setmode 3440 1440')

        state = self._get_state()
        return state, {'i': self.step_i}  # initial state

    def close(self):
        self.game.close()


def doom_test():
    cfg_update = {'mode': Mode.PLAYER}
    doom = DOOM('doom/scenarios/arena_lowered.wad', 'map01', cfg_update)
    game_over = False
    step_times = []
    step_i = 0

    while not game_over and not step_i > 300:
        # a = doom.action_space.sample()
        a = np.array([0, 0, 0]), 0
        state, reward, terminated, truncated, info = doom.step(a)

        step_i = info['i']
        step_times.append(info['step_t'])
        game_over = terminated or truncated

    print(f'Avg step times: {np.mean(step_times):.2f} ({np.std(step_times):.2f}) ms')


if __name__ == '__main__':
    doom_test()
