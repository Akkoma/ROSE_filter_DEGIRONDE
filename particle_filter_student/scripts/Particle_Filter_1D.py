import random


from common.Particle import Particle
from common.ToolBox import distance_to_obstacle,update_coord_according_scale
import math

class Particle_Filter:

    NB_PARTICLES=200
    FIXED_PLANE_Y = 100
    increment = 0
    DISTANCE_ERROR = 2

    width=0
    height=0

    MOTION_PLANNER_MIN=-1
    MOTION_PLANNER_MAX=5

    SCALE_FACTOR=10

    obs_grid=[]
    particle_list=[]


    def __init__(self,width,height,obs_grid):
        self.width=width
        self.height=height
        self.obs_grid=obs_grid
        self.particle_list=self.getRandParticle(self.NB_PARTICLES,0,width,self.FIXED_PLANE_Y,self.FIXED_PLANE_Y)

    def resetParticle(self):
        self.particle_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, self.FIXED_PLANE_Y, self.FIXED_PLANE_Y)

        # ----------------------------------------------------------------------------------------------------------------
        # ----------------------------------------- COMPUTED RANDOM PARTICLES--------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def getRandParticle(self,nbr, start_x, max_x, start_y, max_y):
        particle_list = []
        ###################################
        ##### TODO
        ##   nbr: number fo particles
        ##   start_x: min x possible coordinate
        ##   max_x: max x possible coordinate
        ##   start_y: min y possible coordinate
        ##   max_y: max y possible coordinate
        #####
        ## Use the Particle object to fill the list particle_list
        ##
        for i in range(nbr):
            x = random.randint(start_x, max_x)
            y = random.randint(start_y,max_y)
            particle_list.append(Particle(x,y,0,0))
        return particle_list

        # ----------------------------------------------------------------------------------------------------------------
        # ----------------------------------- UPDATE PARTICLE ACCORDING NEX POSE-----------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def updateParticle(self,plane_pose):
        # process particle according motion planning
        self.particle_list = self.motion_prediction()

        current_distance_to_obstacle = distance_to_obstacle(plane_pose['x'], plane_pose['y'], self.obs_grid,self.width,self.height,self.SCALE_FACTOR)

        self.weightingParticle_list( current_distance_to_obstacle)


        # ----------------------------------------------------------------------------------------------------------------
        # -------------------------------------- MOTION PREDICTION AND RESAMPLING   --------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def motion_prediction(self):
        new_particle_list = []
        choices = {particule.id():particule.w for particule in self.particle_list}

        #Original resampling all particles

        #for _ in range(len(self.particle_list)):
        #    coord = self.weighted_random_choice(choices)
        #    x_coord = int(coord.split('_')[0])
        #    y_coord = int(coord.split('_')[1])
#
        #    # Gaussian motion model (smoother noise)
        #    dx = int(random.gauss(0, 2))  # mean 0, std 2
        #    dy = int(random.gauss(0, 1))  # small lateral noise
#
        #    new_x = max(0, min(self.width, x_coord + dx))
 
        #    new_y = 0
#
        #    new_particle_list.append(Particle(new_x, new_y, 0, 0))

        # Keep a few top particles unchanged

        #Keep top 10% best particles unchanged
        top_particles = sorted(self.particle_list, key=lambda p: p.w, reverse=True)
        elite_count = max(1, len(self.particle_list) // 10)
        for p in top_particles[:elite_count]:
            new_particle_list.append(Particle(p.x, p.y, 0, 0))

        # Resample the rest
        for _ in range(len(self.particle_list) - elite_count):
            coord = self.weighted_random_choice(choices)
            x_coord = int(coord.split('_')[0])
            y_coord = int(coord.split('_')[1])

            dx = random.randint(self.MOTION_PLANNER_MIN, self.MOTION_PLANNER_MAX)
            dy = random.randint(self.MOTION_PLANNER_MIN, self.MOTION_PLANNER_MAX)
            new_x = max(0, min(self.width, x_coord + dx))
            new_y = 0

            new_particle_list.append(Particle(new_x, new_y, 0, 0))

        # Alternative: Original resampling with motion model

        #for i in range(len(self.particle_list)):
        #    

        #    ###################################
        #    ##### TODO
        #    ##   self.particle_list: list of available particles
        #    ##
        #    #####
        #    ## Use the function self.weighted_random_choice(choices) returning
        #    #  coordinate from a particle according a
        #    ##  roulette wheel algorithm
        #    #  Note that weighted_random_choice return a string containing coodinate x and y of the selected particle
        #    #   coord = self.weighted_random_choice(choices)
        #    #   x_coord = int(coord.split('_')[0])
        #    #   y_coord = int(coord.split('_')[1])

        #    coord = self.weighted_random_choice(choices)
        #    x_coord = int(coord.split('_')[0])
        #    y_coord = int(coord.split('_')[1])
        #    #exploration noise
        #    dx = random.randint(self.MOTION_PLANNER_MIN, self.MOTION_PLANNER_MAX)
        #    # keep y mostly fixed (since plane is on fixed line),
        #    # but you could add small noise if needed
        #    dy = 0  

        #    new_x = max(0, min(self.width, x_coord + dx))
        #    new_y = max(0, min(self.height, y_coord + dy))

        #    # Create new particle with neutral weight, prob=0
        #    #new_particle = Particle(new_x, new_y, 1.0, 0.0)
        #    #new_particle_list.append(new_particle)
        #    new_particle_list.append(Particle(new_x, new_y, 0, 0))
        return new_particle_list

        # -------------------------------------------------------
        # ----------- SELECT PARTICLE  -----------
        # -------------------------------------------------------
    def weighted_random_choice(self,choices):
        ###################################
        ##### TODO
        ##   choices: dictionary holding particle coordination as key
        ##  and weight as value
        ##  return the selected particle key
        #####
        particles = list(choices.keys())
        weights = list(choices.values())
        total_weight= sum(weights)
        if total_weight == 0:
            return random.choice(particles)
        r = random.uniform(0, total_weight)
        cumulative = 0.0
        for key, weight in choices.items():
            cumulative += weight
            if r <= cumulative:
                return key

    # ----------------------------------------------------------------------------------------------------------------
    # --------------------------------------------- EVALUATE PARTICLE (proba) ---------------------------------------
    # ----------------------------------------------------------------------------------------------------------------
    def weightingParticle_list(self,observed_distance):
        sum_weights = 0
        for i in range(len(self.particle_list)):
            #Compute individual particle weight
            current_weight = self.weightingParticle(self.particle_list[i].x, self.particle_list[i].y + 50 , observed_distance)
            self.particle_list[i].w = current_weight
            sum_weights += current_weight
        for i in range(len(self.particle_list)):
            if sum_weights != 0:
                #compute proba sucha as weight is normalized
                self.particle_list[i].proba = self.particle_list[i].w / float(sum_weights)
            else:
                self.particle_list[i].proba = 0


    # -----------------------------------------------------
    #  ----------- EVALUATE PARTICLE (Weight)  -----------
    # -----------------------------------------------------


    def weightingParticle(self,p_x, p_y, observed_distance):
        ###################################
        ##### TODO
        ##   p_x: x coordinate of the particle p
        ##  p_y: y coordinate of the particle p
        ##  observed_distance: distance to the ground
        ##  measure by the probe
        ##
        ## return weight corresponding to the given particle
        ## according observation
        ##
        ## Note ue the function distance_to_obstacle to get the
        ## estimate particle to the ground distance
        
        particule_distance_to_obstacle = distance_to_obstacle(p_x, p_y, self.obs_grid, self.width, self.height, self.SCALE_FACTOR)
        
        ## Exponential
        error = abs(observed_distance - particule_distance_to_obstacle)
        # Lambda controls steepness
        lam = 0.5
        weight = math.exp(-lam * error)

        # Gaussian probability density
        #sigma = 2.0  # assumed standard deviation of sensor noise
        #error = observed_distance - particule_distance_to_obstacle
        #
        #weight = math.exp(- (error ** 2) / (2 * sigma ** 2)) / (math.sqrt(2 * math.pi) * sigma)

        #Inverse Distance Error
        #error = abs(observed_distance - particule_distance_to_obstacle)
        
        # Avoid division by zero
        #weight = 1.0 / (1.0 + error)
        
        return weight