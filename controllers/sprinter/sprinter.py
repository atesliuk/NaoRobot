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
        
        
    def run(self, genome, numOfCycles):   
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

        performance = self.calculateDistance(startPoint, self.getCoordinates())        
        return performance
        
    def resetSimulation(self):
        Supervisor.simulationReset(self)

    def getCoordinates(self):
        return Node.getPosition(Supervisor.getSelf(self))

    def calculateDistance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)
        


class GeneticAlgorithm():

    MIN_F = 0 * 100
    MAX_F = 20 * 100
    MIN_ROBOT_HEIGHT = 0 * 100
    MAX_ROBOT_HEIGHT = 2 * 100
    MIN_SHIFT_Y = 0 * 100
    MAX_SHIFT_Y = 1 * 100
    MIN_STEP_HEIGHT = 0 * 100
    MAX_STEP_HEIGHT = 2 * 100
    MIN_STEP_LENGTH = 0 * 100
    MAX_STEP_lENGTH = 1 * 100
    MIN_ARM_SWING = 0 * 100
    MAX_ARM_SWING = 4 * 100

    def __init__(self, initGenome, populationSize, numOfGenerations, numOfCycles):
        self.initGenome = initGenome
        self.populationSize = populationSize
        self.numOfGenerations = numOfGenerations
        self.numOfCycles = numOfCycles
        self.controller = Sprinter()
        self.controller.initialize()
        random.seed(123)        

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

    def start(self):
        self.createInitialPopulation()

        for i in range(self.numOfGenerations):
            print("")
            print("")
            print("================================")
            print("\tGeneration " + str(i+1))
            print("================================")
            self.runGeneration()
            self.breedNewPopulation()

    def runGeneration(self):
        for i in range(len(self.population)):
            genome = self.population[i][0]
            performance = self.controller.run(genome, self.numOfCycles)
            self.population[i]=(genome, performance)
            print("Performance for robot " + str(i+1) + ": " + str(performance))

    def selectGenome(self):
        pass

    

    def breedNewPopulation(self):
        sumOfPerformances = 0
        for i in self.population:
            sumOfPerformances = sumOfPerformances + i[1]
        print("Sum of performances: "+str(sumOfPerformances))
        newPopulation = list()

        for i in range(self.populationSize):
            index1 = self.selectRandomGenome(sumOfPerformances)
            print(index1)
            newPopulation.append((self.population[index1][0], 0))

        self.population = newPopulation        
        
    def selectRandomGenome(self, sumOfPerformances):
        randomNum = float(decimal.Decimal(random.randrange(0, int(sumOfPerformances*100)))/100)

        currGenomeIndex = 0
        sumSoFar = self.population[0][1]
        
        while sumSoFar < randomNum:
            currGenomeIndex = currGenomeIndex + 1
            sumSoFar = sumSoFar + self.population[currGenomeIndex][1]

        return currGenomeIndex
        

        
geneticAlgorithm = GeneticAlgorithm([8, 1, 0.26, 0.5, 0.2, 2], 20, 50, 400)
geneticAlgorithm.start()
