from typing import Optional, Union, List, Tuple

import numpy as np
import gym
from vizdoom import DoomGame, Button, GameVariable, ScreenFormat, ScreenResolution, AutomapMode


# TODO minimize sliding:
#   https://forum.zdoom.org/viewtopic.php?p=933227&sid=fdef5121aec04509a2b76c8048ea5cc5#p933227

# TODO make feedback() used in omnidrive a class object that stores the game setpoint and ways to call for current pos

# TODO LATER: make DOOM a subclass of a abstract class (GAME) to generalize over games
from gym.core import RenderFrame, ActType, ObsType


class DOOM(gym.Env):

    DEFAULT_CFG = {
        'res': ScreenResolution.RES_1920X1080,
        'win_visible': True,
        'render_hud': False,
        'automap': False,
        'buttons': [Button.MOVE_FORWARD_BACKWARD_DELTA, Button.MOVE_LEFT_RIGHT_DELTA, Button.TURN_LEFT_RIGHT_DELTA,
                    Button.ATTACK],
        'ep_timeout': 1000 * 45,  # in ticks
        'skiprate': 4,
        'living_r': -.1 / 4. / 2.,
        'shot_r': -10,
        'dmg_taken_r': -1,  # / hp
        'max_player_speed': 420,  # cm/s; rat max is 500
    }

    def __init__(self, wad_path, map_id, cfg=DEFAULT_CFG):
        super().__init__()

        self.game = DoomGame()
        self.game.set_doom_scenario_path(wad_path)
        self.game.set_doom_map(map_id)

        self.game.set_screen_resolution(cfg['res'])
        self.game.set_screen_format(ScreenFormat.RGB24)
        self.game.set_render_hud(cfg['render_hud'])
        # self.game.add_game_args("-width 3440 -height 1440")  # -record recordings
        self.game.add_game_args("+fullscreen 1 +viz_nocheat 0")
        # self.game.add_game_args('+snd_mididevice -1 +snd_midipatchset /media/viktor/OS/csiki/rats_play_doom/doom/gm.dls')
        self.game.set_sound_enabled(True)  # TODO crashes because of old 1.19 openal version that sticks to vizdoom somewhow

        self.game.set_render_crosshair(False)
        self.game.set_render_weapon(True)
        self.game.set_render_decals(True)
        self.game.set_render_particles(True)

        self.game.set_automap_buffer_enabled(cfg['automap'])
        self.game.set_automap_rotate(False)
        self.game.set_automap_render_textures(False)
        self.game.set_automap_mode(AutomapMode.OBJECTS_WITH_SIZE)

        buttons = cfg['buttons']
        for b in buttons:
            self.game.add_available_button(b)

        vars_needed = [GameVariable.AMMO3, GameVariable.DAMAGE_TAKEN, GameVariable.HEALTH,
                       GameVariable.POSITION_X, GameVariable.POSITION_Y, GameVariable.ANGLE,
                       GameVariable.VELOCITY_X, GameVariable.VELOCITY_Y,
                       GameVariable.USER1, GameVariable.USER2, GameVariable.FRAGCOUNT, GameVariable.HITCOUNT,
                       GameVariable.ATTACK_READY]
        for v in vars_needed:
            self.game.add_available_game_variable(v)

        self.game.set_episode_timeout(cfg['ep_timeout'])
        self.game.set_episode_start_time(10)
        self.game.set_window_visible(cfg['win_visible'])

        self.game.set_living_reward(cfg['living_r'])
        self.game.set_console_enabled(True)

        self.game.init()

        # after-init settings
        self.game.set_console_enabled(True)
        if cfg['automap']:
            self.game.send_game_command('am_showmonsters true')
            self.game.send_game_command('am_colorset 2')

        # extra command settings
        self.game.send_game_command('sv_cheats 1')
        # self.game.send_game_command('fov 120')  # has to be called below for some reason

        # movement: https://www.reddit.com/r/Doom/comments/4nt3fo/i_got_bored_and_did_some_math_on_the_original/
        #   8 map units = 1 foot ~= 30 cm
        #   35 tics = 1 sec
        #   max speed originally is 30 mu/tic
        self.cm_per_map_unit = 30.48 / 8.
        move_speed_rng = cfg['max_player_speed'] / self.cm_per_map_unit
        move_turn_rng = 100  # TODO
        move_space = gym.spaces.Box(low=np.array([-move_speed_rng, -move_speed_rng, -move_turn_rng]),
                                    high=np.array([move_speed_rng, move_speed_rng, move_turn_rng]))
        shoot_space = gym.spaces.Discrete(1)
        self.action_space = gym.spaces.Tuple([move_space, shoot_space])
        # self.observation_space =  # TODO img + imp shot + whatever; see game_variables in rat_vr1

    def step(self, action: ActType) -> Tuple[ObsType, float, bool, bool, dict]:
        pass

    def render(self) -> Optional[Union[RenderFrame, List[RenderFrame]]]:
        pass

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None) -> Tuple[ObsType, dict]:
        # TODO before
        self.game.new_episode()
        # TODO

    def close(self):
        pass
