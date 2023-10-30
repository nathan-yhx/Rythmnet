import scipy.spatial.distance
from PIL import Image, ImageTk
import numpy as np
from tkinter import Button, Canvas, Menu, Label, StringVar, filedialog
import os


class Pedestrian:
    """
    Defines a single pedestrian.
    """

    def __init__(self, position, desired_speed):
        self.diagonal = [0,0,0,0]
        self._position = position
        self._desired_speed = desired_speed
        self.delay = True 
        self.delaytime = 0



    @property
    def position(self):
        return self._position

    @property
    def desired_speed(self):
        return self._desired_speed

    def get_neighbors(self, scenario):
        """
        Compute all neighbors in a 9 cell neighborhood of the current position.
        Obstacles are not considered as neighbors, then they are inaccesible.
        :param scenario: The scenario instance.
        :return: A list of neighbor cell indices (x,y) around the current position.
        """
        return [
            (int(x + self._position[0]), int(y + self._position[1]))
            for x in [-1, 0, 1]
            for y in [-1, 0, 1]
            if 0 <= x + self._position[0] < scenario.width and 0 <= y + self._position[1] < scenario.height and np.abs(x) + np.abs(y) > 0 and scenario.grid[x + self._position[0], y + self._position[1]] != Scenario.NAME2ID['OBSTACLE']
        ]
    
    def checking_move(self,scenario):
        if self.delay: 
            neighbors = self.get_neighbors(scenario)
            next_cell_distance = scenario.target_distance_grids[self._position[0]][self._position[1]]

            next_pos = self._position
            for (n_x, n_y) in neighbors:
                if next_cell_distance > scenario.target_distance_grids[n_x, n_y]:
                    next_pos = (n_x, n_y)
            diagonal = abs(next_pos[0] - self._position[0]) - abs(next_pos[1] - self._position[1])
            # if diagonal==0:
            #     self.delaytime = 1.4/ self._desired_speed if diagonal == 0 else 1/self.desired_speed
            #     self.delaytime = 1.4
            #     self.totaltime += 1.4 /self._desired_speed
            # else:
            #     self.delaytime -= 0.01

            # if diagonal != 0 :
            #     self.update_step(self)
            # else:
            #     self.delaytime = 
            self.update_step(self)
            self.delaytime =1.4/ self._desired_speed if diagonal == 0 else 1.0 /self.desired_speed
            self.delay =False
        else:
            self.delaytime -= 0.01
            if round(self.delaytime , 2) <= 0:
                self.update_step(self)

                  
        

    def update_step(self, scenario):
        """
        Moves to the cell with the lowest distance to the target.
        This does not take obstacles or other pedestrians into account.
        Pedestrians can occupy the same cell.

        :param scenario: The current scenario instance.
        """

        neighbors = self.get_neighbors(scenario)
        next_cell_distance = scenario.target_distance_grids[self._position[0]][self._position[1]]
        # print(next_cell_distance)

        next_pos = self._position
        minDistance = next_cell_distance
        tmp_next_pos = next_pos
        tmp_next_cell_distance = next_cell_distance
        for (n_x, n_y) in neighbors:
            
            if minDistance > scenario.target_distance_grids[n_x, n_y]:
                minDistance = scenario.target_distance_grids[n_x, n_y]
                tmp_next_pos = (n_x, n_y)
                tmp_next_cell_distance = scenario.target_distance_grids[n_x, n_y]

        # print(tmp_next_pos,self.position)
        # print(n_x,self._position[0], n_y,self._position[1])
        n_x,n_y = tmp_next_pos
        flag = 0
        if n_x + 1 == self._position[0] and n_y+1 == self._position[1]:
                    self.diagonal[0]+=1/1.41 
                    if self.diagonal[0] >= 1:
                        flag=1
                        self.diagonal[0] -= 1
        elif n_x + 1 == self._position[0] and n_y-1 == self._position[1]:
                    self.diagonal[1]+=1/1.41 
                    if self.diagonal[1] >= 1:
                        flag=1
                        self.diagonal[1] -= 1
        elif n_x - 1 == self._position[0] and n_y+1 == self._position[1]:
                    self.diagonal[2]+=1/1.41 
                    if self.diagonal[2] >= 1:
                        flag=1
                        self.diagonal[2] -= 1
        elif n_x - 1 == self._position[0] and n_y-1 == self._position[1]:
                    self.diagonal[3]+=1/1.41 
                    if self.diagonal[3] >= 1:
                        flag=1
                        self.diagonal[3] -= 1
        else:
            flag=1
        
        if flag == 1:
            next_pos = tmp_next_pos
            next_cell_distance = tmp_next_cell_distance  
            scenario.grid[self._position[0], self._position[1]] = Scenario.NAME2ID['VISITED']
            self._position = next_pos

        #Update position
        


class Scenario:
    """
    A scenario for a cellular automaton.
    """
    GRID_SIZE = (500, 500)
    ID2NAME = {
        0: 'EMPTY',
        1: 'TARGET',
        2: 'OBSTACLE',
        3: 'PEDESTRIAN',
        4: 'VISITED'
    }
    NAME2COLOR = {
        'EMPTY': (255, 255, 255),
        'PEDESTRIAN': (255, 0, 0),
        'TARGET': (0, 0, 255),
        'OBSTACLE': (255, 0, 255),
        'VISITED': (208, 208, 208)
    }
    NAME2ID = {
        ID2NAME[0]: 0,
        ID2NAME[1]: 1,
        ID2NAME[2]: 2,
        ID2NAME[3]: 3,
        ID2NAME[4]: 4
    }

    def __init__(self, width, height):
        if width < 1 or width > 1024:
            raise ValueError(f"Width {width} must be in [1, 1024].")
        if height < 1 or height > 1024:
            raise ValueError(f"Height {height} must be in [1, 1024].")

        self.width = width
        self.height = height
        self.grid_image = None
        self.grid = np.zeros((width, height))
        self.pedestrians = []
        self.target_distance_grids = self.recompute_target_distances()

    def recompute_target_distances(self):
        self.target_distance_grids = self.update_target_grid()
        return self.target_distance_grids

    def update_target_grid(self):
        """
        Computes the shortest distance from every grid point to the nearest target cell.
        This does not take obstacles into account.
        :returns: The distance for every grid cell, as a np.ndarray.
        """
        targets = []
        for x in range(self.width):
            for y in range(self.height):
                if self.grid[x, y] == Scenario.NAME2ID['TARGET']:
                    targets.append([y, x])  # y and x are flipped because they are in image space.
        if len(targets) == 0:
            return np.zeros((self.width, self.height))

        targets = np.row_stack(targets)
        x_space = np.arange(0, self.width)
        y_space = np.arange(0, self.height)
        xx, yy = np.meshgrid(x_space, y_space)
        positions = np.column_stack([xx.ravel(), yy.ravel()])

        # after the target positions and all grid cell positions are stored,
        # compute the pair-wise distances in one step with scipy.
        distances = scipy.spatial.distance.cdist(targets, positions)
        #print(distances)

        # now, compute the minimum over all  distances to all targets.
        distances = np.min(distances, axis=0)

        return distances.reshape((self.width, self.height))

    def update_step(self):
        """
        Updates the position of all pedestrians.
        This does not take obstacles or other pedestrians into account.
        Pedestrians can occupy the same cell.
        """
        for pedestrian in self.pedestrians:
            pedestrian.update_step(self)

    @staticmethod
    def cell_to_color(_id):
        return Scenario.NAME2COLOR[Scenario.ID2NAME[_id]]

    def target_grid_to_image(self, canvas, old_image_id):
        """
        Creates a colored image based on the distance to the target stored in
        self.target_distance_gids.
        :param canvas: the canvas that holds the image.
        :param old_image_id: the id of the old grid image.
        """
        im = Image.new(mode="RGB", size=(self.width, self.height))
        pix = im.load()
        for x in range(self.width):
            for y in range(self.height):
                target_distance = self.target_distance_grids[x][y]
                pix[x, y] = (max(0, min(255, int(10 * target_distance) - 0 * 255)),
                             max(0, min(255, int(10 * target_distance) - 1 * 255)),
                             max(0, min(255, int(10 * target_distance) - 2 * 255)))
        im = im.resize(Scenario.GRID_SIZE, Image.NONE)
        self.grid_image = ImageTk.PhotoImage(im)
        canvas.itemconfigure(old_image_id, image=self.grid_image)

    def to_image(self, canvas, old_image_id):
        """
        Creates a colored image based on the ids stored in self.grid.
        Pedestrians are drawn afterwards, separately.
        :param canvas: the canvas that holds the image.
        :param old_image_id: the id of the old grid image.
        """
        im = Image.new(mode="RGB", size=(self.width, self.height))
        pix = im.load()
        for x in range(self.width):
            for y in range(self.height):
                pix[x, y] = self.cell_to_color(self.grid[x, y])
        for pedestrian in self.pedestrians:
            x, y = pedestrian.position
            pix[x, y] = Scenario.NAME2COLOR['PEDESTRIAN']
        im = im.resize(Scenario.GRID_SIZE, Image.NONE)
        self.grid_image = ImageTk.PhotoImage(im)
        canvas.itemconfigure(old_image_id, image=self.grid_image)


