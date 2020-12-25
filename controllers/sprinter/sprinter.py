# A simple open loop controller generating walking behavior 
# on a humanoid robot NAO.
# We use the asumption of 'parallel kinematics', i.e., feet are 
# kept parallel to the ground, to simplify kinematic calculations.
# The motion dynamics is generated with sin/cos based oscillations.

# Necessary imports
from controller import Robot
from controller import Supervisor
from controller import Node
import math
import random
import decimal

# Sprinter class represents the Nao robot in the simulation and handles all interactions
# with the robot and the environment
class Sprinter(Supervisor):

    # Initialization, seting up the robot and the environment. Get device pointers, enable 
    # sensors and set robot initial pose
    def initialize(self):
        # This is the time step (ms)
        self.timeStep = int(self.getBasicTimeStep())
        
        # Get pointers to the shoulder motors.
        self.RShoulderPitch = self.getDevice('RShoulderPitch')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')
        
        # Get pointers to the 12 motors of the legs
        self.RHipYawPitch = self.getDevice('RHipYawPitch')
        self.LHipYawPitch = self.getDevice('LHipYawPitch')
        self.RHipRoll     = self.getDevice('RHipRoll')
        self.LHipRoll     = self.getDevice('LHipRoll')
        self.RHipPitch    = self.getDevice('RHipPitch')
        self.LHipPitch    = self.getDevice('LHipPitch')
        self.RKneePitch   = self.getDevice('RKneePitch')
        self.LKneePitch   = self.getDevice('LKneePitch')
        self.RAnklePitch  = self.getDevice('RAnklePitch')
        self.LAnklePitch  = self.getDevice('LAnklePitch')
        self.RAnkleRoll   = self.getDevice('RAnkleRoll')
        self.LAnkleRoll   = self.getDevice('LAnkleRoll')

        # Get pointer to the robot's head pitch sensor
        self.HeadPitchS = self.getDevice('HeadPitchS')
        self.HeadPitchS.enable(True)
               
        
    # Move the left foot (keep the foot paralell to the ground)
    def left(self, x, y, z):
        # x, z 
        self.LKneePitch.setPosition ( z      )
        self.LHipPitch.setPosition  (-z/2 + x)
        self.LAnklePitch.setPosition(-z/2 - x)
        
        # y
        self.LHipRoll.setPosition  ( y)
        self.LAnkleRoll.setPosition(-y)

    
    # Move the right foot (keep the foot paralell to the ground)
    def right(self, x, y, z):
        # x, z
        self.RKneePitch.setPosition ( z      )
        self.RHipPitch.setPosition  (-z/2 + x)
        self.RAnklePitch.setPosition(-z/2 - x)
        
        # y
        self.RHipRoll.setPosition  ( y)
        self.RAnkleRoll.setPosition(-y)
        

    # Launch simulation of the robot walking    
    def run(self, genome, numOfCycles, strategy):
        self.strategy = strategy

        # Reseting simulation, so each robot starts from the same position
        Supervisor.simulationReset(self)
        
        # Decoding genome into parameters
        f            = genome[0]
        robot_height = genome[1]
        shift_y      = genome[2]
        step_height  = genome[3]
        step_length  = genome[4]
        arm_swing    = genome[5]
        
        # Coordinates of the robot's starting point
        startPoint = [-5.345, 0.334, -0.65]

        # Counter of cycles
        currentCycle = 0

        # While goal number of simulation cycles is not reached, robot will walk
        while self.step(self.timeStep) != -1 and currentCycle < numOfCycles:
            # Increasing nr. of current cycle
            currentCycle = currentCycle + 1

            # Scale the time to modulate the frequency of the walk
            t = self.getTime()*f
            
            # Calculating y variable
            yLeftRight = math.sin(t)*shift_y
            
            # Calculating x variable
            zLeft  = (math.sin(t)          + 1.0) / 2.0 * step_height + robot_height
            zRight = (math.sin(t + math.pi)+ 1.0) / 2.0 * step_height + robot_height

            # Calculating x varaibles
            # math.sin(t + math.pi/2) = math.cos(t)
            xLeft  = math.cos(t          )*step_length
            xRight = math.cos(t + math.pi)*step_length
            
            # Applying calculated variables to robot
            self.left(  xLeft, yLeftRight, zLeft )
            self.right(xRight, yLeftRight, zRight)
            
            # Move shoulders to stabilize steps
            self.RShoulderPitch.setPosition(arm_swing*xLeft  + math.pi/2  -0.1)
            self.LShoulderPitch.setPosition(arm_swing*xRight + math.pi/2  -0.1)

            # If robot falls, the simulation is finished (only for strategies 1 and 2)
            if (abs(self.HeadPitchS.getValue()) > 5E-7 and self.strategy != 3):
                break

        # Calculating and returning performance of the robot
        performance = self.calculateDistance(startPoint, self.getCoordinates())     
        return performance

    # Function that returns coordinates of the robot's location in the environment
    def getCoordinates(self):
        return Node.getPosition(Supervisor.getSelf(self))

    # Function that calculates distance between two given points (depends on strategy)
    # See description of each strategy at the end of the document
    def calculateDistance(self, point1, point2):
        performance = 0
        if (self.strategy == 1):
            # For strategy 1, only forward movement matters
            performance = point2[0] - point1[0]
            if (performance < 0):
                performance = 0
        else:
            # Euclidean distance
            performance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
        return performance
        

# GeneticAlgorithm class performs the whole evolutionary computation process. It sets up
# the environment, creates initial population with random genomes, launches many simulations
# where the robots with different genomes are tested and evaluated, and breeds new population 
# with help of cross-over and mutations 
class GeneticAlgorithm():

    # Constants. Values of the robot's parameters can be within those ranges
    MIN_F = 0.0
    MAX_F = 20.0
    MIN_ROBOT_HEIGHT = 0.0
    MAX_ROBOT_HEIGHT = 2.0
    MIN_SHIFT_Y = 0.0
    MAX_SHIFT_Y = 1.0
    MIN_STEP_HEIGHT = 0.0
    MAX_STEP_HEIGHT = 2.0
    MIN_STEP_LENGTH = 0.0
    MAX_STEP_lENGTH = 1.0
    MIN_ARM_SWING = 0.0
    MAX_ARM_SWING = 4.0

    # Constants for cross-over and mutation
    # Probability of the genome to get flipped over during cross-over
    CROSSOVER_FLIPPING_RATE = 1.0/6
    # Probability of the mutation
    MUTATION_RATE = 1.0/6
    # How large there can be deviation in the parameter during the mutation
    MUTATION_MAGNITUDE = 0.05
    # How much % of the population's best genomes will get passed to the next generation
    ELITISM_FRACTION = 0.05

    # Different patterns of genomes. They will be used for benchmarking 
    SLOW_GENOME = [4.0, 0.5, 0.3, 0.4, 0.2, 2.0]
    WALK_IT_GENOME = [8.0, 1.0, 0.26, 0.5, 0.25, 2.0]
    LARGER_STEPS_GENOME = [10.0, 0.9, 0.2, 0.5, 0.25, 2.0]
    TIPPEL_TAPPEL_GENOME = [16.0, 1.0, 0.1, 0.5, 0.2, 2.0]
    RUN_GENOME = [14.0, 0.9, 0.15, 0.7, 0.4, 1.5]

    # Initialization of GeneticAlgorithm object. Setting up the environment, creating a controller (a Robot),
    # creating files for logging 
    def __init__(self, populationSize, numOfGenerations, numOfCycles, strategy, controller = None):
        # Assigning passed values
        self.populationSize = populationSize
        self.numOfGenerations = numOfGenerations
        self.numOfCycles = numOfCycles
        self.strategy = strategy
        self.controller = controller

        # If controller wasn't passed, that means that it has not been yet created and the application should 
        # create and initialize one
        if (self.controller == None):
            self.controller = Sprinter()
            self.controller.initialize()

        # These lists hold minimum and maximum allowed values for each part of the genome. Will be used in mutation
        self.PARAMS_MIN_VALUES = [self.MIN_F, self.MIN_ROBOT_HEIGHT, self.MIN_SHIFT_Y, self.MIN_STEP_HEIGHT, self.MIN_STEP_LENGTH, self.MIN_ARM_SWING]
        self.PARAMS_MAX_VALUES = [self.MAX_F, self.MAX_ROBOT_HEIGHT, self.MAX_SHIFT_Y, self.MAX_STEP_HEIGHT, self.MAX_STEP_lENGTH, self.MAX_ARM_SWING]

        # Seeding (so randomly generated values could be replicated during testing)
        random.seed(123)

        # Creating logging files
        self.averagePerformanceFile = open(".averagePerformance_" + str(self.strategy) + ".txt", "w")
        self.bestPerformanceFile = open(".bestPerformance_" + str(self.strategy) + ".txt", "w")        

        self.averagePerformanceFile.write("Generation\tPerformance" + "\n")
        self.bestPerformanceFile.write("Generation\tPerformance\tGenome" + "\n")

        # Initializing cash
        self.cash = dict()

    # Generating random genomes for the initial population
    def createInitialPopulation(self):
        # List that stores population (population's genomes) as well as their performance
        self.population = list()

        # Loop that creates random genomes (taking into account ranges in which each particular parameter should be)
        for i in range(self.populationSize):
            # Creating new genome. Iterating through each part of the genome
            newGenome = list()
            for j in range(6):
                # Taking minimum and maximum allowed values, and generating a random value within that range
                randomValue = float(decimal.Decimal(random.randrange(self.PARAMS_MIN_VALUES[j] * 10000, self.PARAMS_MAX_VALUES[j] * 10000))/10000)
                newGenome.append(randomValue)
            self.population.append((newGenome,0))


    # This method launches simulations in which performance of benchmark genome patterns get tested and recorded
    def testPatterns(self):
        # Testing genome patterns in the simulation
        print("Testing patterns ...")
        self.performance_slow = self.controller.run(self.SLOW_GENOME, self.numOfCycles, self.strategy)
        print("'Slow' robot performance: " + str(self.performance_slow))

        self.performance_walk_it = self.controller.run(self.WALK_IT_GENOME, self.numOfCycles, self.strategy)
        print("'Walk it' robot performance: " + str(self.performance_walk_it))

        self.performance_larger_steps = self.controller.run(self.LARGER_STEPS_GENOME, self.numOfCycles, self.strategy)
        print("'Larger steps' robot performance: " + str(self.performance_larger_steps))

        self.performance_tippel_tappel = self.controller.run(self.TIPPEL_TAPPEL_GENOME, self.numOfCycles, self.strategy)
        print("'Tippel Tappel' robot performance: " + str(self.performance_tippel_tappel))
        
        self.performance_run = self.controller.run(self.RUN_GENOME, self.numOfCycles, self.strategy)
        print("'Running' robot performance: " + str(self.performance_run))
        
        # Writting down the results in the file
        self.patternsPerformanceFile = open(".patternsPerformance_" + str(self.strategy) + ".txt", "w")
        self.patternsPerformanceFile.write("Pattern\tPerformance" + "\n")
        self.patternsPerformanceFile.write("SLOW_GENOME\t" + str(self.performance_slow) + "\n")
        self.patternsPerformanceFile.write("WALK_IT_GENOME\t" + str(self.performance_walk_it) + "\n")
        self.patternsPerformanceFile.write("LARGER_STEPS_GENOME\t" + str(self.performance_larger_steps) + "\n")
        self.patternsPerformanceFile.write("TIPPEL_TAPPEL_GENOME\t" + str(self.performance_tippel_tappel) + "\n")
        self.patternsPerformanceFile.write("RUN_GENOME\t" + str(self.performance_run) + "\n")
        self.patternsPerformanceFile.close()
        
        print("Testing finished\n")


    # Starting and running the whole process of the genetic algorithm
    def start(self):
        # At first initial population is created
        self.createInitialPopulation()

        # Then there is a loop where in each cycle one generation get tested and the next one gets created
        for i in range(self.numOfGenerations):
            print("\n\n================================")
            print("\tGeneration " + str(i+1))
            print("================================")
            # Running simulation where performance of all members of the population gets tested
            self.runGeneration()
            # After that new generation is created
            self.breedNewPopulation(i)

        # After the whole genetic algorithm is finished, the application should wrap up some processes        
        self.finishingProgram()
        # Since only one Robot (controller) can be initialized in the population, we should return it
        # so we can reuse it next time (when we launch another genetic algorithm (with different strategy))
        return self.controller


    # Closing opened files    
    def finishingProgram(self):
        self.averagePerformanceFile.close()
        self.bestPerformanceFile.close()        
        print("All done")


    # The method launches simulation for each memebr of the population, as well as records their performance
    def runGeneration(self):
        # Iterating through each genome in the population
        for i in range(len(self.population)):
            genome = self.population[i][0]
            # Running the simulation and recording the performance. If the genome is in the cash (the same genome has been run 
            # before) then we don't need to run it again since we already know its performance
            performance = 0
            strGenome = str(genome)
            if (strGenome in self.cash):
                performance = self.cash[strGenome]
            else:
                performance = self.controller.run(genome, self.numOfCycles, self.strategy)
                self.cash[strGenome] = performance
            
            self.population[i]=(genome, performance)
            print("Performance for robot " + str(i+1) + ": " + str(performance) + ", genome: " + str(genome))


    # The method creates new population for the next generation
    def breedNewPopulation(self, generationNr):

        # This function is used for sorting the genomes in the list by their performance (so we can easily select best ones (elitism))
        def sortFunction(e):
            return e[1]

        # Recording the best performance for the generation
        self.population.sort(reverse=True, key=sortFunction)
        self.bestPerformanceFile.write(str(generationNr) + "\t" + str(self.population[0][1]) + "\t" + str(self.population[0][0]) + "\n")

        # Calculating sum of the performances in the current generation (will be used for calculating average performance as well as in 
        # roulette selection)
        sumOfPerformances = 0
        for i in self.population:
            sumOfPerformances = sumOfPerformances + i[1]
        print("Sum of performances: "+str(sumOfPerformances))
        self.averagePerformanceFile.write(str(generationNr) + "\t" + str(sumOfPerformances/self.populationSize) + "\n")

        # Creating list for the next generation
        newPopulation = list()

        # Selecting fraction of the best genomes to add them to the next generation (elitism)               
        numOfEliteGenomes = int(round(self.populationSize * self.ELITISM_FRACTION))
        for i in range(numOfEliteGenomes):
            newPopulation.append((self.population[i][0], 0))
        
        # Generate rest population with help of crossover and mutation
        for i in range(numOfEliteGenomes, self.populationSize):
            # Selecting two parent with help of Roulette Selection
            index1 = self.rouletteSelection(sumOfPerformances)
            index2 = self.rouletteSelection(sumOfPerformances)
            # Making an offspring from those two parents (cross-over)
            newGenome = self.crossover(index1, index2)
            # Applying mutation to the offspring
            newGenome = self.mutate(newGenome)
            # Adding the offspring to the new generation's population
            newPopulation.append((newGenome, 0))
        # Once population for the new generation is created, it replaces the old generation
        self.population = newPopulation        


    # Roulette selection    
    def rouletteSelection(self, sumOfPerformances):
        # Generating a random number from 0 to the sum of the performances
        randomNum = float(decimal.Decimal(random.randrange(0, int(sumOfPerformances*100)))/100)
        # Adding genomes' performances one by one until the sum becomes greater than the random 
        # number. Last added genome is the selected one 
        currGenomeIndex = 0
        sumSoFar = self.population[0][1]        
        while sumSoFar < randomNum:
            currGenomeIndex = currGenomeIndex + 1
            sumSoFar = sumSoFar + self.population[currGenomeIndex][1]
        return currGenomeIndex

    # Multiple point crossover of two parents, returns a newly generated offspring
    def crossover(self, index1, index2):
        # Taking two genomes that were selected
        genome1 = self.population[index1][0]
        genome2 = self.population[index2][0]
        # Initializing empty genome for the new offspring
        newGenome = list()
        # This value tells from which genome to copy values
        usingGenome1 = True
        # Iterating through each part of the genome
        for i in range(6):
            # Generating a random number to test whether a flip over should happen
            randomNumber = float(decimal.Decimal(random.randrange(0, 10000))/10000)
            # If a randomly generated number is less than the flipping rate, then
            # the flip over happens (another one genome is used for coppying)
            if (randomNumber < self.CROSSOVER_FLIPPING_RATE):
                usingGenome1 = not usingGenome1

            # Values from a parent's genome get copied to the offspring's genome
            if usingGenome1:
                newGenome.append(genome1[i])
            else:
                newGenome.append(genome2[i])
        return newGenome


    # Mutation of the passed genome
    def mutate(self, genome):
        # Iterating through the each part of the passed genome
        for i in range(6):
            # Generating a random decimal number between 0 and 1
            randomNumber = float(decimal.Decimal(random.randrange(0, 10000))/10000)
            # If the random number is less than the mutation rate, then the nutation should happen
            if (randomNumber < self.MUTATION_RATE):
                # Calculating allowed borders for the mutate value (based on mutation magnitude, as well as on borders of 
                # the specific parameter that is stored in that place in genome)
                allowedDeviation = (self.PARAMS_MAX_VALUES[i] - self.PARAMS_MIN_VALUES[i]) * self.MUTATION_MAGNITUDE
                minValue =  genome[i] - allowedDeviation
                maxValue = genome[i] + allowedDeviation

                # Checking that the minimum and maximum values are within borders of the allowed range
                if (minValue < self.PARAMS_MIN_VALUES[i]):
                    minValue = self.PARAMS_MIN_VALUES[i]
                if (maxValue > self.PARAMS_MAX_VALUES[i]):
                    maxValue = self.PARAMS_MAX_VALUES[i]

                # Calculating new value within specific borders
                genome[i] = round(float(decimal.Decimal(random.randrange(round(minValue*10000), round(maxValue*10000))/10000)),4)
        return genome



""" Running the whole application 3 times - once with each strategy"""

# Strategy 1 - best performing robot is the one that goes fastest straight forward and doesn't fall
# Strategy 2 - best performing robot is the one that goes fastest in any direction and doesn't fall
# Strategy 3 - best performing robot is the one that moves fastest in any direction (can fall and crawl)

print("************* RUNNING STRATEGY 1 *************\n")
geneticAlgorithm = GeneticAlgorithm(populationSize=200, numOfGenerations=50, numOfCycles=500, strategy=1)
geneticAlgorithm.testPatterns()
currController = geneticAlgorithm.start()

print("\n************* RUNNING STRATEGY 2 *************\n")
geneticAlgorithm = GeneticAlgorithm(populationSize=200, numOfGenerations=50, numOfCycles=500, strategy=2, controller=currController)
geneticAlgorithm.testPatterns()
geneticAlgorithm.start()

print("\n************* RUNNING STRATEGY 3 *************\n")
geneticAlgorithm = GeneticAlgorithm(populationSize=200, numOfGenerations=50, numOfCycles=500, strategy=3, controller=currController)
geneticAlgorithm.testPatterns()
geneticAlgorithm.start()
