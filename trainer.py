
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import vizdoom
from DOOM import DOOM


class Trainer:
    def __init__(self, cspace_path=None, omni_drive=None):
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

        grid_ids = [self._closest_grid_node(p) for pi, p in enumerate(poz)]
        edges = list(set((grid_ids[pi], grid_ids[pi + 1]) for pi in range(poz.shape[0] - 1)
                         if grid_ids[pi] != grid_ids[pi + 1]))
        nodes = [(cxi, cyi) for cxi, cyi in np.ndindex(node_poz.shape[:2])]
        for node, pos in zip(nodes, node_poz.reshape(-1, 2)):
            self.cspace.add_node(node, pos=pos)
        self.cspace.add_edges_from(edges)

        # test path planning and draw coordinate space
        start_pos, end_pos = poz[0], poz[len(poz) // 2]
        path = self._plan_path(start_pos, end_pos)

        nx.draw(self.cspace, pos=node_poz, node_size=50, node_color='grey')
        nx.draw(self.cspace, nodelist=path, pos=node_poz, node_size=60, node_color='red')
        nx.draw(self.cspace, nodelist=[self._closest_grid_node(start_pos), self._closest_grid_node(end_pos)],
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

    def _closest_grid_node(self, pos: np.ndarray):
        dist = np.linalg.norm(self.cspace.graph['node_poz'] - pos, axis=-1)
        return np.unravel_index(np.argmin(dist), self.cspace.graph['node_poz'].shape[:2])

    @staticmethod
    def _path_plan_dist_heuristic(a, b):
        x1, y1 = a
        x2, y2 = b
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def _plan_path(self, start_pos, end_pos):
        start_node = self._closest_grid_node(start_pos)
        end_node = self._closest_grid_node(end_pos)
        return nx.astar_path(self.cspace, start_node, end_node, heuristic=ArenaTrainer._path_plan_dist_heuristic)


class ArenaTrainer(Trainer):
    def __init__(self, cspace_path=None, omni_drive=None, reward_freq=50,
                 kill_r=1., in_sight_r=.1, approach_r=.1, in_sight_dist=200, too_close_dist=50):
        super().__init__(cspace_path, omni_drive)
        self.reward_freq = reward_freq
        self.kill_r = kill_r
        self.in_sight_r = in_sight_r
        self.approach_r = approach_r
        self.in_sight_dist = in_sight_dist
        self.too_close_dist = too_close_dist

        self.kill_count = 0
        self.closest_to_monster = np.inf
        self.best_angle = False

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

        # check if monster killed
        if state.kill_count > self.kill_count:
            self._monster_reset(state.kill_count)
            return self.kill_r

        # check if monster got approached or turned towards
        r = 0.
        if state.monsters_present >= 1 and step_i % self.reward_freq == 0:

            # reward approach
            monster_dist = ArenaTrainer._monster_dist(state)
            if self.closest_to_monster == np.inf:
                self.closest_to_monster = monster_dist
            elif monster_dist < self.closest_to_monster:  # got closer
                self.closest_to_monster = monster_dist
                r += self.approach_r if monster_dist > self.too_close_dist else 0  # reward only up to a distance

            # reward turn towards monster - got in sight when close enough
            if monster_dist < self.in_sight_dist:
                monster_angle = ArenaTrainer._monster_sight_angle(state)
                if monster_angle < self.best_angle:
                    self.best_angle = monster_angle
                    r += self.in_sight_r
            else:  # got out of range for in sight turn reward
                self.best_angle = min(180, self.best_angle + 2)  # prevents obvious reward milking

            print('player: ', state.position_x, state.position_y, state.angle)
            print('monster:', state.monster_pos_x, state.monster_pos_y)
            print('distance:', self._monster_dist(state))

        return r  # returns reward in [0, 1]

    def enforce_action(self, step_i, state):
        pass  # TODO


class DiscoveryTrainer(Trainer):
    def __init__(self, cspace_path=None, omni_drive=None):
        super().__init__(cspace_path, omni_drive)

    def give_reward(self, step_i, state):
        pass  # TODO

    def enforce_action(self, step_i, state):
        pass  # TODO


if __name__ == '__main__':
    trainer = ArenaTrainer(cspace_path='arena_lowered.map01.pckl')
    # grid_size = 30
    # trainer.man_explore_cspace(grid_size, 'doom/scenarios/arena_lowered.wad', 'map01')
    trainer.test_trainer('doom/scenarios/arena_lowered.wad', 'map01')
