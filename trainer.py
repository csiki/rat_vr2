# TODO first implement ArenaTrainer, then a generalized trainer
# TODO implement DiscoveryTrainer

# TODO implement it so you can use multiple trainers simultaneously
# TODO each trainer when trainer.step() is called, pass a flag whether it can start an active training (e.g. rolling) process,
#   also every trainer returns whether it is currently running active training

# TODO can create a MultiTrainer class that can incorporate any trainers, and their priorities, so if 2 trainers
#   want to train simultaneously, then one is preferred over the other; also it can weight/sum rewards coming from all...
import time

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import vizdoom
from DOOM import DOOM


class ArenaTrainer:
    def __init__(self, cspace_path=None):
        self.cspace: nx.Graph = None if cspace_path is None else nx.read_gpickle(cspace_path)

    def man_explore_cspace(self, grid_size, wad_path, map_id, cfg_update=None, no_action_limit=50, cspace_path=None):
        cfg_update = {} if cfg_update is None else cfg_update
        cfg_update['mode'] = vizdoom.Mode.SPECTATOR
        doom = DOOM(wad_path, map_id, cfg_update)

        game_over = False
        xs, ys, ts = [], [], []
        no_action_counter = 0

        while not game_over:  # and not step_i > 3000:
            # a = doom.action_space.sample()
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

        node_poz = np.meshgrid(np.arange(min_pos[0] + grid_size // 2, max_pos[0] + grid_size // 2, grid_size),
                                      np.arange(min_pos[1] + grid_size // 2, max_pos[1] + grid_size // 2, grid_size))
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

    def _closest_grid_node(self, pos):
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


if __name__ == '__main__':
    trainer = ArenaTrainer(cspace_path='arena_lowered.map01.pckl')
    grid_size = 30
    trainer.man_explore_cspace(grid_size, 'doom/scenarios/arena_lowered.wad', 'map01')
