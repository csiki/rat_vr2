import time

from typing import Callable, Any, Union, Tuple, List

import keyboard
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import vizdoom
from collections import namedtuple
from omni_drive import OmniDrive, local_man_drive
from player_movement import PlayerMovement, Feedback
from DOOM import DOOM
from lever import Lever


class Trainer:
    def __init__(self, cspace_path: str = None, omni_drive: OmniDrive = None):
        self.cspace: nx.Graph = None if cspace_path is None else nx.read_gpickle(cspace_path)
        self.omni_drive = omni_drive

    def give_reward(self, step_i, state):
        raise NotImplemented

    def enforce_action(self, step_i, state):
        raise NotImplemented

    def man_explore_cspace(self, grid_size: float, wad_path: str, map_id: str, cfg_update: dict = None,
                           no_action_limit=50, cspace_path: str = None):
        cfg_update = {} if cfg_update is None else cfg_update
        cfg_update['mode'] = vizdoom.Mode.SPECTATOR
        doom = DOOM(wad_path, map_id, cfg_update)

        game_over = False
        xs, ys, ts = [], [], []
        no_action_counter = 0

        while not game_over:  # and not step_i > 3000:
            a = np.array([0, 0, 0]), 0
            state, reward, terminated, truncated, info = doom.step(a)
            step_i = info['i']
            no_action = sum(info['action']) == 0

            if not no_action:
                ts.append(step_i)
                xs.append(state.position_x)
                ys.append(state.position_y)
                no_action_counter = 0
            else:
                no_action_counter += 1

            game_over = terminated or truncated or no_action_counter > no_action_limit

        doom.close()

        ts, xs, ys = map(np.asarray, [ts, xs, ys])
        poz = np.stack([xs, ys], axis=1)

        # build grid
        min_pos = np.min(poz, axis=0)
        max_pos = np.max(poz, axis=0)

        node_poz = np.meshgrid(np.arange(min_pos[0] + grid_size / 2, max_pos[0] + grid_size / 2, grid_size),
                               np.arange(min_pos[1] + grid_size / 2, max_pos[1] + grid_size / 2, grid_size))
        node_poz = np.stack(node_poz, axis=-1)
        self.cspace = nx.Graph(node_poz=node_poz)  # all info stored in coordinate space graph

        grid_ids = [self.closest_grid_node(p) for pi, p in enumerate(poz)]
        edges = list(set((grid_ids[pi], grid_ids[pi + 1]) for pi in range(poz.shape[0] - 1)
                         if grid_ids[pi] != grid_ids[pi + 1]))
        nodes = [(cxi, cyi) for cxi, cyi in np.ndindex(node_poz.shape[:2])]
        for node, pos in zip(nodes, node_poz.reshape(-1, 2)):
            self.cspace.add_node(node, pos=pos)
        self.cspace.add_edges_from(edges)

        # test path planning and draw coordinate space
        start_pos, end_pos = poz[0], poz[len(poz) // 2]
        path = self.plan_path(start_pos, end_pos)

        nx.draw(self.cspace, pos=node_poz, node_size=50, node_color='grey')
        nx.draw(self.cspace, nodelist=path, pos=node_poz, node_size=60, node_color='red')
        nx.draw(self.cspace, nodelist=[self.closest_grid_node(start_pos), self.closest_grid_node(end_pos)],
                pos=node_poz, node_size=70, node_color='black')
        plt.title('Coordinate space')
        plt.show()

        # save coordinate space graph
        cspace_path = wad_path[max(wad_path.rfind('/'), wad_path.rfind('\\')) + 1:wad_path.rfind('.')] + f'.{map_id}.pckl' \
            if cspace_path is None else cspace_path
        nx.write_gpickle(self.cspace, cspace_path)
        print('Saved coordinate space to', cspace_path)

    def test_trainer(self, wad_path: str, map_id: str, cfg_update: dict = None, no_action_limit=300,
                     call_reward=True, call_enforce=False, print_freq=50):
        cfg_update = {} if cfg_update is None else cfg_update
        cfg_update['mode'] = vizdoom.Mode.SPECTATOR
        doom = DOOM(wad_path, map_id, cfg_update)

        game_over = False
        no_action_counter = 0
        accum_reward = 0

        while not game_over:  # and not step_i > 3000:
            a = np.array([0, 0, 0]), 0
            state, reward, terminated, truncated, info = doom.step(a)
            step_i = info['i']
            no_action = sum(info['action']) == 0

            if call_reward:
                accum_reward += self.give_reward(step_i, state)
            if call_enforce:
                self.enforce_action(step_i, state)
            if step_i % print_freq == 0:
                print(f'@{step_i} Accum Reward = {accum_reward}')

            no_action_counter = no_action_counter + 1 if no_action else 0
            game_over = terminated or truncated or no_action_counter > no_action_limit

        doom.close()

    def closest_grid_node(self, pos: Union[tuple, np.ndarray], node_subset=None):

        # closest node in a subgraph
        if node_subset:
            nodes_w_poz = nx.get_node_attributes(self.cspace.subgraph(node_subset), 'pos')
            node_ids = list(nodes_w_poz.keys())
            node_poz = np.stack(nodes_w_poz.values())
            dist = np.linalg.norm(node_poz - np.asarray(pos), axis=-1)
            return node_ids[np.argmin(dist)]

        # sped up version when looking for closest node on whole graph
        else:
            dist = np.linalg.norm(self.cspace.graph['node_poz'] - np.asarray(pos), axis=-1)
            return np.unravel_index(np.argmin(dist), self.cspace.graph['node_poz'].shape[:2])

    @staticmethod
    def _2d_euclidean_dist(a, b):
        x1, y1 = a
        x2, y2 = b
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def plan_path(self, start_pos, end_pos):
        start_node = self.closest_grid_node(start_pos)
        end_node = self.closest_grid_node(end_pos)
        return nx.astar_path(self.cspace, start_node, end_node, heuristic=ArenaTrainer._2d_euclidean_dist)


class ArenaTrainer(Trainer):

    # TRAIN_STATES = ['away', 'close', 'in-sight']  # , 'shot'

    def __init__(self, cspace_path=None, omni_drive=None, lever=None, player_mov=None, reward_freq=50,
                 kill_r=1., in_sight_r=.1, approach_r=.1, close_dist=200, too_close_dist=50, in_sight_deg=8,
                 enforce_after_no_action=None, enforce_after_no_reward=None):
        super().__init__(cspace_path, omni_drive)
        self.lever = lever
        self.player_mov = player_mov

        # reward vars
        self.reward_freq = reward_freq
        self.kill_r = kill_r
        self.in_sight_r = in_sight_r
        self.approach_r = approach_r
        self.close_dist = close_dist
        self.too_close_dist = too_close_dist
        self.in_sight_deg = in_sight_deg

        self.kill_count = 0
        self.closest_to_monster = np.inf
        self.best_angle = False
        self.last_movement_t = time.time()
        self.last_pos = (0, 0)
        self.last_reward_t = time.time()
        self.enforcing_from = 'aimless'  # don't enforce anything by default

        # enforcing vars
        self.train_state_i = 0  # away
        self.enforce_after_no_action = enforce_after_no_action  # in s; enforcing move after this many seconds of no mov
        self.enforce_after_no_reward = enforce_after_no_reward  # in s; same but after lack of reward
        self.enforce_fun = None

    def _monster_reset(self, new_kill_count):
        self.kill_count = new_kill_count
        self.closest_to_monster = np.inf
        self.best_angle = 180

    @staticmethod
    def _monster_dist(state):
        return np.sqrt((state.position_x - state.monster_pos_x) ** 2 + (state.position_y - state.monster_pos_y) ** 2)

    @staticmethod
    def _angle_between_vectors3(A, B, V):  # chatgpt
        import math
        # Vector pointing from A to B
        AB = [B[0] - A[0], B[1] - A[1]]
        # dot product of V and AB
        dot_product = V[0] * AB[0] + V[1] * AB[1]
        # Magnitude of vectors
        mag_V = np.sqrt(V[0] ** 2 + V[1] ** 2)
        mag_AB = np.sqrt(AB[0] ** 2 + AB[1] ** 2)
        # angle between vectors
        angle = np.arccos(dot_product / (mag_V * mag_AB)) * 180 / np.pi
        return angle

    @staticmethod
    def _monster_sight_angle(state):
        player = [state.position_x, state.position_y]  # player
        monster = [state.monster_pos_x, state.monster_pos_y]
        sight = [np.cos(state.angle / 180 * np.pi), np.sin(state.angle / 180 * np.pi)]
        return ArenaTrainer._angle_between_vectors3(player, monster, sight)

    def give_reward(self, step_i, state):
        r = 0.

        # check if monster killed
        if state.kill_count > self.kill_count:
            self._monster_reset(state.kill_count)
            r = self.kill_r

        # check if monster got approached or turned towards
        elif state.monsters_present > 0 and step_i % self.reward_freq == 0:

            # reward approach
            monster_dist = ArenaTrainer._monster_dist(state)
            if self.closest_to_monster == np.inf:
                self.closest_to_monster = monster_dist
            elif monster_dist < self.closest_to_monster:  # got closer
                self.closest_to_monster = monster_dist
                r += self.approach_r if monster_dist > self.too_close_dist else 0  # reward only up to a distance

            # reward turn towards monster - got in sight when close enough
            if monster_dist < self.close_dist:
                monster_angle = ArenaTrainer._monster_sight_angle(state)
                if monster_angle < self.best_angle:
                    self.best_angle = monster_angle
                    r += self.in_sight_r
            else:  # got out of range for in sight turn reward
                self.best_angle = min(180, self.best_angle + 2)  # prevents obvious reward milking

            print('player: ', state.position_x, state.position_y, state.angle)
            print('monster:', state.monster_pos_x, state.monster_pos_y)
            print('distance:', self._monster_dist(state))

        self.last_reward_t = time.time() if r > 0 else self.last_reward_t
        return r  # returns reward in [0, 1]

    def _get_train_state(self, state):
        train_state = 'aimless'
        if state.monsters_present > 0:
            train_state = 'away'
            if ArenaTrainer._monster_dist(state) < self.close_dist:
                train_state = 'close'
                if ArenaTrainer._monster_sight_angle(state) < self.in_sight_deg:
                    train_state = 'in-sight'
        return train_state

    def _get_exec_path_fun(self, start_state, path) -> Callable[[Any], Any]:
        # if not rolling: find closest grid point on path, roll ball to the next point
        def _fun(state):
            if self.omni_drive.mounted and not self.omni_drive.rolling:
                closest_on_path = self.closest_grid_node((state.position_x, state.position_y), path)
                closest_on_path_i = path.index(closest_on_path)

                # at the end of path, just move to monster
                if closest_on_path_i == len(path) - 1:
                    fb = Feedback(self.player_mov, setpoint=np.array([state.monster_pos_x, state.monster_pos_y,
                                                                      state.angle]))  # keep current angle for now
                # on path
                else:
                    goal = self.cspace.nodes[path[closest_on_path_i + 1]]['pos']
                    fb = Feedback(self.player_mov, setpoint=np.concatenate([goal, [state.angle]]))

                self.omni_drive.roll(fb, eps=self.too_close_dist / 2)

        return _fun

    def _get_exec_turn_fun(self, start_state) -> Callable[[Any], Any]:
        def _fun(state):
            if self.omni_drive.mounted and not self.omni_drive.rolling:
                fb = Feedback(self.player_mov, rel_setpoint=np.array([0, 0, self._monster_sight_angle(state)]))  # TODO maybe the negative of the sight angle, or should it be in rad?
                self.omni_drive.roll(fb, eps=self.in_sight_deg / 2)

        return _fun

    def _get_exec_kill(self, start_state) -> Callable[[Any], Any]:
        def _fun(state):
            self.lever.pull()  # TODO depends on lever interface
        return _fun

    def enforce_action(self, step_i, state):  # TODO !!! test
        pos = (state.position_x, state.position_y)
        self.last_movement_t = time.time() if self.last_pos != pos else self.last_movement_t
        train_state = self._get_train_state(state)

        # transition to aimless resets enforcement
        #   typically when the monster is killed, so there's nothing to shoot
        if train_state == 'aimless':
            self.enforcing_from = 'aimless'
            self.enforce_fun = None
            self.omni_drive.letgo()

        # initialise enforcement: when starting enforcement or switching to a different enforcement state
        may_start_enforcement = self.enforcing_from == 'aimless' and train_state != 'aimless'
        new_enforcement_state = self.enforcing_from != 'aimless' and self.enforcing_from != train_state
        if may_start_enforcement or new_enforcement_state:
            init_enforce = (self.enforce_after_no_action is not None and
                            time.time() - self.last_movement_t > self.enforce_after_no_action) or \
                           (self.enforce_after_no_reward is not None and
                            time.time() - self.last_reward_t > self.enforce_after_no_reward)
            init_enforce = init_enforce or new_enforcement_state  # start new enforcing step (without stopping)

            if init_enforce:
                self.enforcing_from = train_state
                self.omni_drive.mount()

                if train_state == 'away':
                    path = self.plan_path((state.position_x, state.position_y),
                                          (state.monster_pos_x, state.monster_pos_y))
                    self.enforce_fun = self._get_exec_path_fun(state, path)

                elif train_state == 'close':
                    self.enforce_fun = self._get_exec_turn_fun(state)

                elif train_state == 'in-sight':
                    self.enforce_fun = self._get_exec_kill(state)

        # run enforcement function
        if self.enforcing_from != 'aimless' and self.enforce_fun is not None:
            self.enforce_fun(state)


class DiscoveryTrainer(Trainer):
    def __init__(self, cspace_path=None, omni_drive=None):
        super().__init__(cspace_path, omni_drive)

    def give_reward(self, step_i, state):
        pass  # TODO

    def enforce_action(self, step_i, state):
        pass  # TODO


class ManualTrainer(Trainer):

    KILL_HAPPENS_WITHIN = 3  # sec after lever pull

    def __init__(self, game: DOOM, omni_drive: OmniDrive, lever: Lever, move_r_per_sec: float, kill_r: float,
                 r_in_every: float, min_r_given: float = 10., omni_speed=.7):
        super().__init__(cspace_path=None, omni_drive=omni_drive)
        self.game = game
        self.omni_drive = omni_drive
        self.lever = lever
        self.move_r_per_sec = move_r_per_sec
        self.kill_r = kill_r
        self.r_in_every = r_in_every  # sec
        self.min_r_given = min_r_given

        self.man_drive = None
        self.enforce_called = False
        self.current_drive = np.zeros(OmniDrive.AXES)
        self.lever_pulled_at = time.time() - 2 * ManualTrainer.KILL_HAPPENS_WITHIN
        self.last_move_rewarded = time.time()
        self.move_r = 0.
        self.kill_count = 0

        # use self.game.set_mode() to set it to vizdoom.Mode.SPECTATOR when rat is under control
        #   every enforce_action() call check keyboard key presses, and if pressed space it changes to control mode
        #   when enforcing movement and kill, give immediate reward according to move_reward_per_sec and kill_reward
        #   check when kill was done and give the reward as given
        #   have keys to increase/decrease speed

    def _setup_man_drive(self):
        self.man_drive = local_man_drive(self.omni_drive)  # omni_drive should be a PiOmniDrive

    def give_reward(self, step_i, state):
        r = 0.
        if np.any(self.current_drive != 0):
            self.move_r += np.linalg.norm(self.current_drive) * self.move_r_per_sec

        if self.move_r > self.min_r_given and time.time() - self.last_move_rewarded > self.r_in_every:
            r += self.move_r * self.r_in_every
            self.move_r = 0
            self.last_move_rewarded = time.time()

        if state.kill_count > self.kill_count and \
                time.time() - self.lever_pulled_at < ManualTrainer.KILL_HAPPENS_WITHIN:
            self.lever_pulled_at = time.time() - 2 * ManualTrainer.KILL_HAPPENS_WITHIN
            r += self.kill_r

        return r

    def enforce_action(self, step_i, state):
        if not self.enforce_called:  # first call
            self._setup_man_drive()
            self.enforce_called = True

        self.current_drive, mount_state = self.man_drive()

        if mount_state == 'mounted':
            self.game.game.set_mode(vizdoom.Mode.SPECTATOR)
        elif mount_state == 'letgo':
            self.game.game.set_mode(vizdoom.Mode.PLAYER)

        if keyboard.is_pressed(Lever.KEY_SHOOT):
            self.lever.pull()
            self.lever_pulled_at = time.time()


if __name__ == '__main__':
    trainer = ArenaTrainer(cspace_path='arena_lowered.map01.pckl')
    # grid_size = 30
    # trainer.man_explore_cspace(grid_size, 'doom/scenarios/arena_lowered.wad', 'map01')
    trainer.test_trainer('doom/scenarios/arena_lowered.wad', 'map01')
