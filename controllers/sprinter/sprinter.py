# A simple open loop controller generating walking behavior 
# on a humanoid robot NAO.
# We use the asumption of 'parallel kinematics', i.e., feet are 
# kept parallel to the ground, to simplify kinematic calculations.
# The motion dynamics is generated with sin/cos based oscillations.


from controller import Robot
from controller import Supervisor
from controller import Node
import math
import random
import decimal

class Sprinter(Supervisor):

    def initialize(self):
        
        """Get device pointers, enable sensors and set robot initial pose."""
        
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

        # Sensors
        self.HeadPitchS = self.getDevice('HeadPitchS')
        self.HeadPitchS.enable(True)
               
        
    # move the left foot (keep the foot paralell to the ground)
    def left(self, x, y, z):
        # x, z 
        self.LKneePitch.setPosition ( z      )
        self.LHipPitch.setPosition  (-z/2 + x)
        self.LAnklePitch.setPosition(-z/2 - x)
        
        # y
        self.LHipRoll.setPosition  ( y)
        self.LAnkleRoll.setPosition(-y)
    
    
    # move the right foot (keep the foot paralell to the ground)
    def right(self, x, y, z):
        # x, z
        self.RKneePitch.setPosition ( z      )
        self.RHipPitch.setPosition  (-z/2 + x)
        self.RAnklePitch.setPosition(-z/2 - x)
        
        # y
        self.RHipRoll.setPosition  ( y)
        self.RAnkleRoll.setPosition(-y)
        
        
    def run(self, genome, numOfCycles, strategy):
        self.strategy = strategy   
        self.resetSimulation()
        
        # f            = 8
        # robot_height = 1.0
        # shift_y      = 0.26
        # step_height  = 0.5
        # step_length  = 0.2
        # arm_swing    = 2.0

        f            = genome[0]
        robot_height = genome[1]
        shift_y      = genome[2]
        step_height  = genome[3]
        step_length  = genome[4]
        arm_swing    = genome[5]
        
        startPoint = [-5.345, 0.334, -0.65]
        i = 0

        while self.step(self.timeStep) != -1 and i < numOfCycles:
            i = i + 1

            # scale the time to modulate the frequency of the walk
            t = self.getTime()*f
            
            # y
            yLeftRight = math.sin(t)*shift_y
            
            # z
            zLeft  = (math.sin(t)          + 1.0) / 2.0 * step_height + robot_height
            zRight = (math.sin(t + math.pi)+ 1.0) / 2.0 * step_height + robot_height

            # x
            # math.sin(t + math.pi/2) = math.cos(t)
            xLeft  = math.cos(t          )*step_length
            xRight = math.cos(t + math.pi)*step_length
            
            # apply
            self.left(  xLeft, yLeftRight, zLeft )
            self.right(xRight, yLeftRight, zRight)
            
            # move shoulders to stabilize steps
            self.RShoulderPitch.setPosition(arm_swing*xLeft  + math.pi/2  -0.1)
            self.LShoulderPitch.setPosition(arm_swing*xRight + math.pi/2  -0.1)

            # If robot falls
            if (abs(self.HeadPitchS.getValue()) > 5E-7 and self.strategy != 3):
                break

        performance = self.calculateDistance(startPoint, self.getCoordinates())     
        return performance
        
    def resetSimulation(self):
        Supervisor.simulationReset(self)

    def getCoordinates(self):
        return Node.getPosition(Supervisor.getSelf(self))

    def calculateDistance(self, point1, point2):
        performance = 0
        if (self.strategy == 1):
            performance = point2[0] - point1[0]
            if (performance < 0):
                performance = 0
        else:
            performance = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
        return performance
        


class GeneticAlgorithm():

    MIN_F = 0.0 * 100
    MAX_F = 20.0 * 100
    MIN_ROBOT_HEIGHT = 0.0 * 100
    MAX_ROBOT_HEIGHT = 2.0 * 100
    MIN_SHIFT_Y = 0.0 * 100
    MAX_SHIFT_Y = 1.0 * 100
    MIN_STEP_HEIGHT = 0.0 * 100
    MAX_STEP_HEIGHT = 2.0 * 100
    MIN_STEP_LENGTH = 0.0 * 100
    MAX_STEP_lENGTH = 1.0 * 100
    MIN_ARM_SWING = 0.0 * 100
    MAX_ARM_SWING = 4.0 * 100
    RANGES = []

    CROSSOVER_FLIPPING_RATE = 1.0/6
    MUTATION_RATE = 1.0/6
    MUTATION_MAGNITUDE = 0.05
    ELITISM_FRACTION = 0.05

    SLOW_GENOME = [4.0, 0.5, 0.3, 0.4, 0.2, 2.0]
    WALK_IT_GENOME = [8.0, 1.0, 0.26, 0.5, 0.25, 2.0]
    LARGER_STEPS_GENOME = [10.0, 0.9, 0.2, 0.5, 0.25, 2.0]
    TIPPEL_TAPPEL_GENOME = [16.0, 1.0, 0.1, 0.5, 0.2, 2.0]
    RUN_GENOME = [14.0, 0.9, 0.15, 0.7, 0.4, 1.5]

    def __init__(self, populationSize, numOfGenerations, numOfCycles, strategy, controller = None):
        self.populationSize = populationSize
        self.numOfGenerations = numOfGenerations
        self.numOfCycles = numOfCycles
        self.strategy = strategy
        self.controller = controller
        if (self.controller == None):
            self.controller = Sprinter()
            self.controller.initialize()
        self.RANGES = [
            (self.MAX_F - self.MIN_F)/100,
            (self.MAX_ROBOT_HEIGHT - self.MIN_ROBOT_HEIGHT)/100,
            (self.MAX_SHIFT_Y - self.MIN_SHIFT_Y)/100,
            (self.MAX_STEP_HEIGHT - self.MIN_STEP_HEIGHT)/100,
            (self.MAX_STEP_lENGTH - self.MIN_STEP_LENGTH)/100,
            (self.MAX_ARM_SWING - self.MIN_ARM_SWING)/100]
        random.seed(123)

        self.averagePerformanceFile = open(".averagePerformance_" + str(self.strategy) + ".txt", "w")
        self.bestPerformanceFile = open(".bestPerformance_" + str(self.strategy) + ".txt", "w")        

        self.averagePerformanceFile.write("Generation\tPerformance" + "\n")
        self.bestPerformanceFile.write("Generation\tPerformance\tGenome" + "\n")

    def createInitialPopulation(self):
        self.population = list()
        for i in range(self.populationSize):
            newGenome = [
                float(decimal.Decimal(random.randrange(self.MIN_F, self.MAX_F))/100),
                float(decimal.Decimal(random.randrange(self.MIN_ROBOT_HEIGHT, self.MAX_ROBOT_HEIGHT))/100),
                float(decimal.Decimal(random.randrange(self.MIN_SHIFT_Y, self.MAX_SHIFT_Y))/100),
                float(decimal.Decimal(random.randrange(self.MIN_STEP_HEIGHT, self.MAX_STEP_HEIGHT))/100),
                float(decimal.Decimal(random.randrange(self.MIN_STEP_LENGTH, self.MAX_STEP_lENGTH))/100),
                float(decimal.Decimal(random.randrange(self.MIN_ARM_SWING, self.MAX_ARM_SWING))/100)
            ]
            self.population.append((newGenome,0))

    def testPatterns(self):
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
        
        self.patternsPerformanceFile = open(".patternsPerformance_" + str(self.strategy) + ".txt", "w")
        self.patternsPerformanceFile.write("Pattern\tPerformance" + "\n")
        self.patternsPerformanceFile.write("SLOW_GENOME\t" + str(self.performance_slow) + "\n")
        self.patternsPerformanceFile.write("WALK_IT_GENOME\t" + str(self.performance_walk_it) + "\n")
        self.patternsPerformanceFile.write("LARGER_STEPS_GENOME\t" + str(self.performance_larger_steps) + "\n")
        self.patternsPerformanceFile.write("TIPPEL_TAPPEL_GENOME\t" + str(self.performance_tippel_tappel) + "\n")
        self.patternsPerformanceFile.write("RUN_GENOME\t" + str(self.performance_run) + "\n")
        self.patternsPerformanceFile.close()
        
        print("Testing finished")
        print("")


    def start(self):
        self.createInitialPopulation()
        for i in range(self.numOfGenerations):
            print("")
            print("")
            print("================================")
            print("\tGeneration " + str(i+1))
            print("================================")
            self.runGeneration()
            self.breedNewPopulation(i)        
        self.finishingProgram()
        return self.controller
        
    def finishingProgram(self):
        self.averagePerformanceFile.close()
        self.bestPerformanceFile.close()        
        print("All done")

    def runGeneration(self):
        for i in range(len(self.population)):
            genome = self.population[i][0]
            performance = self.controller.run(genome, self.numOfCycles, self.strategy)
            self.population[i]=(genome, performance)
            print("Performance for robot " + str(i+1) + ": " + str(performance) + ", genome: " + str(genome))

    def breedNewPopulation(self, generationNr):

        def sortFunction(e):
            return e[1]

        sumOfPerformances = 0
        for i in self.population:
            sumOfPerformances = sumOfPerformances + i[1]
        print("Sum of performances: "+str(sumOfPerformances))
        self.averagePerformanceFile.write(str(generationNr) + "\t" + str(sumOfPerformances/self.populationSize) + "\n")

        self.population.sort(reverse=True, key=sortFunction)
        self.bestPerformanceFile.write(str(generationNr) + "\t" + str(self.population[0][1]) + "\t" + str(self.population[0][0]) + "\n")

        # Select 10% of the best genomes (elitism)
        newPopulation = list()        
        numOfEliteGenomes = int(round(self.populationSize * self.ELITISM_FRACTION))
        for i in range(numOfEliteGenomes):
            newPopulation.append((self.population[i][0], 0))
        
        # Generate the rest 90% by crossover and mutation
        for i in range(numOfEliteGenomes, self.populationSize):
            index1 = self.rouletteSelection(sumOfPerformances)
            index2 = self.rouletteSelection(sumOfPerformances)
            newGenome = self.crossover(index1, index2)
            newGenome = self.mutate(newGenome)
            newPopulation.append((newGenome, 0))
        self.population = newPopulation        
        
    def rouletteSelection(self, sumOfPerformances):
        randomNum = float(decimal.Decimal(random.randrange(0, int(sumOfPerformances*100)))/100)
        currGenomeIndex = 0
        sumSoFar = self.population[0][1]        
        while sumSoFar < randomNum:
            currGenomeIndex = currGenomeIndex + 1
            sumSoFar = sumSoFar + self.population[currGenomeIndex][1]
        return currGenomeIndex

    def crossover(self, index1, index2):
        # Crossover of two parents, returns a newly generated offspring
        genome1 = self.population[index1][0]
        genome2 = self.population[index2][0]
        newGenome = list()
        usingGenome1 = True
        for i in range(6):
            randomNumber = float(decimal.Decimal(random.randrange(0, 10000))/10000)
            if (randomNumber < self.CROSSOVER_FLIPPING_RATE):
                usingGenome1 = not usingGenome1
            if usingGenome1:
                newGenome.append(genome1[i])
            else:
                newGenome.append(genome2[i])
        return newGenome

    def mutate(self, genome):
        # mutate the passed genome
        for i in range(6):
            randomNumber = float(decimal.Decimal(random.randrange(0, 10000))/10000)
            if (randomNumber < self.MUTATION_RATE):
                deviationRange = self.RANGES[i]
                deviation = round(float(decimal.Decimal(random.randrange(0, deviationRange*self.MUTATION_MAGNITUDE*200))/100) - deviationRange,4)
                genome[i] = genome[i] + deviation
        return genome
        
# Strategy 1 - best performing robot is the one that goes fastest straight forward and doesn't fall
# Strategy 2 - best performing robot is the one that goes fastes in any direction and doesn't fall
# Strategy 3 - best performing robot is the one that moves fastest in any direction (can fall and crawl)

print("************* RUNNING STRATEGY 1 *************")
print(" ")        
geneticAlgorithm = GeneticAlgorithm(populationSize=200, numOfGenerations=50, numOfCycles=500, strategy=1)
geneticAlgorithm.testPatterns()
currController = geneticAlgorithm.start()

print(" ")
print("************* RUNNING STRATEGY 2 *************")
print(" ")
geneticAlgorithm = GeneticAlgorithm(populationSize=200, numOfGenerations=50, numOfCycles=500, strategy=2, controller=currController)
geneticAlgorithm.testPatterns()
geneticAlgorithm.start()

print(" ")
print("************* RUNNING STRATEGY 3 *************")
print(" ")
geneticAlgorithm = GeneticAlgorithm(populationSize=200, numOfGenerations=50, numOfCycles=500, strategy=3, controller=currController)
geneticAlgorithm.testPatterns()
geneticAlgorithm.start()
