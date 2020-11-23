#!/usr/bin/env python3
from grid_cell import GridCell

class GridMap:
    def __init__(self):
        self._grids = []

    def add_grid_cell(self):
        self._grids.append(GridCell())


if __name__ == '__main__':
    grid_map = GridMap()
    grid_map.add_grid_cell()

