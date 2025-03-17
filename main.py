import pygame
import os
import time
import math
from collections import deque

def clear():
    if os.name == "nt": 
        os.system("cls")
    else:
        os.system("clear")

BLACK = (0, 0, 0)
RED = (255, 0 ,0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
BROWN = (80, 80, 80)
WHITE = (255,255,255)
LIGHTGREY = (150, 150, 150)
LIGHTBLUE = (0, 0, 150)
GREY = (21, 21, 21)

SECONDS_PER_DAY = 24 * 60 * 60        
SECONDS_PER_YEAR = 365 * SECONDS_PER_DAY  

class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2D({self.x}, {self.y})"

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, other):
        return Vector2D(self.x / other, self.y / other)
    
    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    def tuple(self):
        return (self.x, self.y)
    
    def castInt(self):
        return Vector2D(int(self.x), int(self.y))

    def magnitude(self): 
        return (self.x**2 + self.y**2) ** 0.5

    def squaredMagnitude(self):
        return (self.x**2 + self.y**2)

    def normalise(self): 
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / mag, self.y / mag)

class Body:
    def __init__(self, distance, mass, bodyRadius, colour, bodyName):
        self.position = Vector2D(distance, 0)
        self.mass = mass
        self.velocity = Vector2D(0, 0)
        self.acceleration = Vector2D(0, 0)
        
        self.colour = colour
        self.bodyRadius = int(bodyRadius)
        self.orbitMinDistance = 1e8
        self.orbitPoints = deque(maxlen=10000)
        self.bodyName = bodyName

    def applyForce(self, force):
        self.acceleration += force / self.mass

    def update(self, deltaTime):
        self.velocity += self.acceleration * deltaTime
        self.position += self.velocity * deltaTime
        self.lastAcceleration = self.acceleration
        self.acceleration = Vector2D(0, 0)
    
    def combineColour(self, other):
        red = ((self.colour[0] + other.colour[0])) // 2
        green = ((self.colour[1] + other.colour[1])) // 2
        blue = ((self.colour[2] + other.colour[2])) // 2
        return (red, green, blue)
    
    def calculateOrbitLines(self): 
        if not self.orbitPoints:
            self.orbitPoints.append(self.position)
            return

        lastPoint = self.orbitPoints[-1]
        if (self.position-lastPoint).squaredMagnitude() > self.orbitMinDistance:
            self.orbitPoints.append(self.position)

    def drawOrbitLines(self, screen, offset, scale):
        self.calculateOrbitLines()
        if len(self.orbitPoints) < 2:
            return 0
        step = max(1, 2 ** int(-math.log10(scale) / 3))
        points = list(self.orbitPoints)[::step]
        scaledPoints = [
            ((p.x + offset.x) * scale, (p.y + offset.y) * scale) 
            for p in points
        ]
        scaledPoints.append(((self.position.x + offset.x) * scale, (self.position.y + offset.y) * scale))
        if len(scaledPoints) >= 2:
            pygame.draw.lines(screen, (50, 50, 52), False, scaledPoints, 1)
    
class Simulation:
    def __init__(self, timeStepIndex, G):
        self.timeStepOptions = [1, 60, 60 * 60, 24 * 60 * 60, 30.4 * 24 * 60 * 60] #, 24 * 60 * 365 *60
        self.timeStepIndex = timeStepIndex
        self.timeStep = self.timeStepOptions[timeStepIndex]
        self.G = G
        self.elapsedTime = 0
        self.bodies = []
        self.initBodies()
        self.bodyAmount = len(self.bodies)

    def initBodies(self):
        self.bodies = [
            Body(0, 1.989e30, 696340000, YELLOW, "Sun"),
            Body(57_910_000_000 , 3.301e23, 2439700  , LIGHTGREY, "Mercury"),
            Body(108_210_000_000, 4.87e24 , 6050000  , BROWN, "Venus"),
            Body(149_597_870_000, 5.972e24, 6371000  , BLUE, "Earth"),
            Body(149_597_870_000 + 384_400_000, 7.348e22, 1737400, BROWN, "Moon"),
            Body(227_900_000_000, 6.39e23 , 3389500 , RED, "Mars"),
            Body(778_600_000_000, 1.898e27, 69911000, LIGHTGREY, "Jupiter"),
            Body(1_439_200_000_000, 5.683e26, 58232000, RED, "Saturn"),
            Body(4_471_300_000_000, 1.024e26, 24622000, BLUE, "Neptune"),
            Body(2.9233e12, 8.681e25, 25362000, LIGHTBLUE, "Uranus"),
        ]
        for i in range(1, len(self.bodies)):
            self.bodies[i].velocity = calculateOrbitalVelocity(self.bodies[0], self.bodies[i], self.G)
        self.bodies[4].velocity += calculateOrbitalVelocity(self.bodies[3], self.bodies[4], self.G)

    def step(self, deltaTime):
        bodyCount = len(self.bodies)
        for i in range(bodyCount):
            for j in range(i + 1, bodyCount):
                force = calculateGravitationalForce(self.bodies[i], self.bodies[j], self.G)
                self.bodies[i].applyForce(force)
                self.bodies[j].applyForce(-force)

        for body in self.bodies:
            body.update(self.timeStep * deltaTime)

        i = 0
        while i < self.bodyAmount:
            for j in range(i+1, self.bodyAmount, 1):
                if checkCollision(self.bodies[i], self.bodies[j]):
                    self.combineBodies(self.bodies[i], self.bodies[j])
                    self.bodyAmount = len(self.bodies)
                    i = -1
                    break
            i += 1

        self.elapsedTime += self.timeStep * deltaTime
    
    def clearOrbitPoints(self):
        for body in self.bodies:
            body.orbitPoints = deque(maxlen=10000)

    def combineBodies(self, body1, body2):
        combinedMass = body1.mass + body2.mass
        newPosition = (body1.position * body1.mass + body2.position * body2.mass) / combinedMass
        newVelocity = (body1.velocity * body1.mass + body2.velocity * body2.mass) / combinedMass
        newRadius = int((body1.bodyRadius + body2.bodyRadius) // 2)
        newColour = body1.combineColour(body2)
        newName = f"{body1.bodyName}-{body2.bodyName}"
        newBody = Body(0, combinedMass, newRadius, newColour, newName)
        newBody.position = newPosition
        newBody.velocity = newVelocity

        self.bodies.remove(body1)
        self.bodies.remove(body2)
        self.bodies.append(newBody)

class Camera:
    def __init__(self, simulation, resolution):
        self.simulation = simulation
        self.scale = 3e-10
        self.zoomSmoothing = 0.1
        self.followZooming = 0.2
        self.position = Vector2D(0, 0)
        self.offset = Vector2D(0, 0)
        self.resolution = resolution
        self.targetScale = self.scale
        self.displayCenter = Vector2D(resolution.x / 2, resolution.y / 2)
        self.cameraFollowIndex = 0
        self.lastCameraFollowIndex = 0
        self.targetPixelRadius = 9

    def getCameraCenter(self):
        return Vector2D(self.displayCenter.x / self.scale, self.displayCenter.y / self.scale)

    def zoomToFill(self):
        targetRadius = self.simulation.bodies[self.cameraFollowIndex].bodyRadius
        if targetRadius > 0:
            self.targetScale = self.targetPixelRadius / targetRadius
    
    def updateCamera(self):
        targetPosition = self.simulation.bodies[self.cameraFollowIndex].position

        scaleDifference = abs(self.targetScale - self.scale)
        snapThreshold= max(1e-12, self.targetScale * 0.001)
        if scaleDifference < snapThreshold:
            self.scale = self.targetScale
        else: 
            self.scale += (self.targetScale - self.scale) * self.zoomSmoothing
        
        desiredPosition = self.getCameraCenter() - self.offset - targetPosition

        if self.lastCameraFollowIndex != self.cameraFollowIndex:
            self.position += (desiredPosition - self.position) * self.followZooming
            if (desiredPosition - self.position).magnitude() < 9e9:
                self.position = desiredPosition
                self.lastCameraFollowIndex = self.cameraFollowIndex
            return

        self.position = desiredPosition

class Renderer:
    def __init__(self, simulation, camera, screen, resolution, font):
        self.simulation = simulation
        self.camera = camera
        self.screen = screen
        self.font = font
        self.doArrows = False
        self.doRelativeArrows = False
        self.doOrbitLines = False
        self.resolution = resolution
        self.gridSize = 50
        self.maxArrowLength = 30
        
    def render(self):
        self.screen.fill(BLACK)
        self.camera.updateCamera()
        maxVelocity = max(body.velocity.magnitude() for body in self.simulation.bodies) if self.simulation.bodies else 1
        for body in self.simulation.bodies:
            if self.doOrbitLines:
               body.drawOrbitLines(self.screen, self.camera.position, self.camera.scale)

            self.drawBody(body)

            if self.doArrows and self.doRelativeArrows:
                self.drawArrows(body, self.simulation.bodies[self.camera.cameraFollowIndex].velocity.magnitude(), self.simulation.bodies[self.camera.cameraFollowIndex])    
            elif self.doArrows:
                self.drawArrows(body, maxVelocity)

        self.drawScale()
        self.drawDebugText()
    
    def drawBody(self, body):
        position = (body.position + self.camera.position) * self.camera.scale
        radius = max(int(body.bodyRadius * self.camera.scale), 1)
        pygame.draw.circle(self.screen, body.colour, position.castInt().tuple(), radius)

    def drawDebugText(self):
        elapsed = self.simulation.elapsedTime
        year = int(elapsed // SECONDS_PER_YEAR)
        day = int((elapsed % SECONDS_PER_YEAR) // SECONDS_PER_DAY)
        secondsToday = elapsed % SECONDS_PER_DAY
        hour = int(secondsToday // 3600)
        minute = int((secondsToday % 3600) // 60)
        second = int(secondsToday % 60)
        timeStepText = ["second", "minute", "hour", "day", "month", "year"]

        debugText = [
            f"Scale: {1/self.camera.scale:.2e} meters per pixel",
            f"Time Step: 1 {timeStepText[self.simulation.timeStepIndex]} per second",
            "",
            f"Year(s): {year}",
            f"Day(s): {day}",
            f"Hour(s): {hour}",
            f"Minute(s): {minute}",
            f"Second(s): {second}",
            "",
            "Tracking:",
            f"      Name: {self.simulation.bodies[self.camera.cameraFollowIndex].bodyName}",
            f"      Mass: {self.simulation.bodies[self.camera.cameraFollowIndex].mass:.2e} Kg",
            f"      Velocity: {self.simulation.bodies[self.camera.cameraFollowIndex].velocity.magnitude():.2e} m/s",
            "",
            "[Mouse Wheel/+/-] Zoom (10%)",
            "[Ctrl + Mouse Wheel] Zoom (50%)",
            "[<-/->] Cycle focus",
            "[RMB] Camera panning",
            "[Z] Toggle arrows",
            "[X] Toggle relative arrows",
            "[T] Cycle time step",
            "[C] Reset camera",
            "[F] Zoom to fill",
            "[O] Toggle orbit lines",
            "[P] Reset orbit points",
            "[W/S] Adjust max arrow length (5px)",
            "[Q/E] Adjust focus mass (10%)",
            "[R] Restart"
        ]
        for i, text in enumerate(debugText):
            if text == "":
                continue
            self.drawText(text, (10, 10 + i*20))

    def drawScale(self, maxLength = 300):
        startPos = (20, self.resolution.y - 40)

        unitKm = 1000
        unitLm = 1.799e10

        roundedLengthMetric = calcPixelRoundedLength(maxLength, self.camera.scale , unitKm)
        pixelLengthMetric = int(roundedLengthMetric * self.camera.scale * unitKm) 
        endPosMetric = (20 + pixelLengthMetric, self.resolution.y - 40)

        roundedLengthLm = calcPixelRoundedLength(maxLength, self.camera.scale , unitLm)
        pixelLengthLm = int(roundedLengthLm * self.camera.scale * unitLm)
        endPosLm = (20 + pixelLengthLm, self.resolution.y - 40)

        self.drawLineTicks(startPos, endPosMetric, True)
        self.drawLineTicks(startPos, endPosLm, False)

        pygame.draw.line(self.screen, WHITE, startPos, endPosMetric, 3)
        pygame.draw.line(self.screen, WHITE, startPos, endPosLm, 3)

        self.drawText(f"{roundedLengthMetric} Km", (60, startPos[1] - 25))
        self.drawText(f"{roundedLengthLm} Light Minutes", (60, startPos[1] + 10))

    def drawArrows(self, body, referenceVelocityArrow, referenceBody=None):
        if body == referenceBody:
            return
        
        if referenceBody:
            relativeVelocity = body.velocity - referenceBody.velocity
            relativeAcceleration = body.lastAcceleration - referenceBody.lastAcceleration
        else:
            relativeVelocity = body.velocity
            relativeAcceleration  = body.lastAcceleration

        position = (body.position + self.camera.position) * self.camera.scale
        scaledArrowLength = self.maxArrowLength * (math.sqrt(relativeVelocity.magnitude()) / math.sqrt(referenceVelocityArrow))
        #displayArrowLength = self.maxArrowLength * math.log(1 + relativeVelocity.magnitude()) / math.log(1 + referenceVelocityArrow)
        displayArrowLength = max(self.maxArrowLength * 0.5, min(scaledArrowLength, self.maxArrowLength))             
        arrowSize = max(4, int(displayArrowLength * 0.15))

        accelerationDirection = relativeAcceleration.normalise()
        startAccelerationPosition = position.castInt()
        endAccelertionPosition = (position + accelerationDirection * displayArrowLength).castInt()            
        pygame.draw.line(self.screen, BLUE, startAccelerationPosition.tuple(), endAccelertionPosition.tuple(), 2)
        self.drawArrowHead(accelerationDirection, endAccelertionPosition, BLUE, arrowSize)

        velocityDirection = relativeVelocity.normalise()
        startVelocityPosition = position.castInt()
        endVelocityPosition = (position + velocityDirection * displayArrowLength).castInt()
        pygame.draw.line(self.screen, RED, startVelocityPosition.tuple(), endVelocityPosition.tuple(), 2)
        self.drawArrowHead(velocityDirection, endVelocityPosition, RED, arrowSize)
    

    def drawArrowHead(self, direction, position, colour, arrowSize = 4):
        angle = math.atan2(direction.y, direction.x)
        
        leftAngle = angle + math.pi / 6
        rightAngle = angle - math.pi / 6  
        leftPoint = (position.x - arrowSize * math.cos(leftAngle), position.y - arrowSize * math.sin(leftAngle))
        rightPoint = (position.x - arrowSize * math.cos(rightAngle), position.y - arrowSize * math.sin(rightAngle))
        
        pygame.draw.polygon(self.screen, colour, [position.tuple(), leftPoint, rightPoint])

    def drawLineTicks(self, startPos, endPos, pointsUp, tickLength=10, thickness=3):
        if pointsUp:
            offset = -tickLength
            pygame.draw.line(self.screen, WHITE, startPos, (startPos[0], startPos[1] + offset), thickness)
            pygame.draw.line(self.screen, WHITE, endPos, (endPos[0], endPos[1] + offset), thickness)
        else:
            offset = tickLength
            pygame.draw.line(self.screen, WHITE, endPos, (endPos[0], endPos[1] + offset), thickness)
    def drawText(self, text, position):
        textSurface = self.font.render(text, True, WHITE)
        self.screen.blit(textSurface, position)

class InputHandler:
    def __init__(self, simulation, renderer, camera):
        self.simulation = simulation
        self.renderer = renderer
        self.camera = camera
        self.panning = False

    def processEvents(self):
        ctrlHeld = pygame.key.get_mods() & pygame.KMOD_CTRL

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handleMouseButtonDown(event, ctrlHeld)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.handleMouseButtonUp(event)
            elif event.type == pygame.MOUSEMOTION:
                self.handleMouseMotion(event)
            elif event.type == pygame.KEYDOWN:
                if not self.handleKeyDown(event):
                    return False

        keys = pygame.key.get_pressed()
        self.handleContinuousKeys(keys)
        return True
    
    def handleMouseButtonDown(self, event, ctrlHeld):
        if event.button == 4 or event.button == 132: 
            if ctrlHeld:
                self.camera.targetScale *= 1.5
            else:
                self.camera.targetScale *= 1.1
        elif event.button == 5 or event.button == 133:
            if ctrlHeld:
                self.camera.targetScale /= 1.5
            else:
                self.camera.targetScale /= 1.1
        elif event.button == 3:
            self.panning = True

    def handleMouseButtonUp(self,event):
        if event.button == 3:
            self.panning = False

    def handleMouseMotion(self, event):     
        if self.panning:
            delta = Vector2D(event.rel[0], event.rel[1]) / self.camera.scale
            self.camera.offset = self.camera.offset - delta

    def handleKeyDown(self, event):
        if event.key == pygame.K_RIGHT:
            self.camera.cameraFollowIndex = (self.camera.cameraFollowIndex + 1) % len(self.simulation.bodies)
            self.camera.offset = Vector2D(0, 0)
        elif event.key == pygame.K_LEFT:
            self.camera.cameraFollowIndex = (self.camera.cameraFollowIndex - 1) % len(self.simulation.bodies)
            self.camera.offset = Vector2D(0, 0)
        elif event.key == pygame.K_c:
            self.camera.offset = Vector2D(0, 0)
        elif event.key == pygame.K_t:
            self.simulation.timeStepIndex = (self.simulation.timeStepIndex + 1) % len(self.simulation.timeStepOptions)
            self.simulation.timeStep = self.simulation.timeStepOptions[self.simulation.timeStepIndex]
        elif event.key == pygame.K_z:
            self.renderer.doArrows = not self.renderer.doArrows
        elif event.key == pygame.K_x:
            self.renderer.doRelativeArrows = not self.renderer.doRelativeArrows
        elif event.key == pygame.K_f:
            self.camera.zoomToFill()
            self.camera.offset = Vector2D(0, 0)
        elif event.key == pygame.K_o:
            self.renderer.doOrbitLines = not self.renderer.doOrbitLines 
            self.simulation.clearOrbitPoints()
        elif event.key == pygame.K_p:
            self.simulation.clearOrbitPoints()
        elif event.key == pygame.K_r:
            return False
        return True

    def handleContinuousKeys(self, keys):
        if keys[pygame.K_EQUALS]:
            self.camera.targetScale *= 1.1
        elif keys[pygame.K_MINUS]:
            self.camera.targetScale /= 1.1
        elif keys[pygame.K_w]:
                    self.renderer.maxArrowLength += 5
        elif keys[pygame.K_s]:
                self.renderer.maxArrowLength -= 5
                if self.renderer.maxArrowLength < 5:
                    self.renderer.maxArrowLength = 5
        elif keys[pygame.K_q]:
            self.simulation.bodies[self.camera.cameraFollowIndex].mass /= 1.2
        elif keys[pygame.K_e]:
            self.simulation.bodies[self.camera.cameraFollowIndex].mass *= 1.2

def calculateGravitationalForce(body1, body2, G):
    direction = body2.position - body1.position
    distanceSquared = direction.squaredMagnitude()
    if distanceSquared == 0:
        return Vector2D(0, 0)
    forceMagnitude = (G * body1.mass * body2.mass) / distanceSquared
    forceDirection = direction / (distanceSquared) ** 0.5
    return forceDirection * forceMagnitude

def calculateOrbitalVelocity(center, body, G):
    direction = body.position - center.position
    distance = (direction.x ** 2 + direction.y ** 2) ** 0.5
    if distance == 0:
        return Vector2D(0, 0)
    velocityMagnitude = (G * center.mass / distance) ** 0.5
    velocityDirection = Vector2D(-direction.y, direction.x) / distance
    return velocityDirection * velocityMagnitude

def checkCollision(body1, body2):
    distanceSquared = (body2.position - body1.position).squaredMagnitude()
    collisionDistance = (body1.bodyRadius + body2.bodyRadius)
    return distanceSquared <= collisionDistance**2

def calcPixelRoundedLength(maxLength, scale, unitScaler):
    maxLength = 200
    realLength = (maxLength / scale) / unitScaler
    magnitude = 10 ** int(math.log10(realLength ))
    return  round(realLength / magnitude) * magnitude 

def chooseResolution():
    resolutionOptions = [Vector2D(1280, 720), Vector2D(1920, 1080), Vector2D(2560, 1440), Vector2D(3840, 2160)]
    print("Select Resolution (1, 2, etc.)\nPress enter for default")
    for i in range(len(resolutionOptions)):
        print(f"[{i+1}] {resolutionOptions[i]}")
    selection = input()
    if selection == "":
        return Vector2D(1920, 1000) 
    else:
        selection = int(selection)
        return resolutionOptions[selection - 1]
     
def main(resolution):
    clear()

    pygame.init()
    pygame.display.set_caption("Planet Sim")
    screen = pygame.display.set_mode(resolution.tuple(), pygame.SCALED | pygame.NOFRAME)
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()
    frameRate = 60

    sim = Simulation(3, 6.67430e-11)
    camera = Camera(sim, resolution)
    renderer = Renderer(sim, camera, screen, resolution, font)
    inputHandler = InputHandler(sim, renderer, camera)

    running = True
    while running:
        frameTime = clock.tick(frameRate) / 1000
        running = inputHandler.processEvents()
        sim.step(frameTime)
        renderer.render()
        pygame.display.flip()

        print(f"FPS: {clock.get_fps():.2f}")

    pygame.quit()

clear()
res = chooseResolution()
while True:
    main(res)