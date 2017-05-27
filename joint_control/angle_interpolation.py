'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.firstTime = 0 
        self.lastTime = 0 

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        (names, times, keys) = keyframes

        timeInKeyframes = self.currentTime - self.firstTime
        
        for nameIndex in range(0, len(names)):
            name = names[nameIndex]
            timeForName = times[nameIndex]
            keyForName = keys[nameIndex]

            if not name in self.perception.joint:
                continue

            
            if timeInKeyframes < timeForName[0]:

                P0 = (timeInKeyframes, self.perception.joint[name])
                P1 = (timeForName[0] + keyForName[0][1][1], keyForName[0][0] + keyForName[0][1][2])
                P2 = (timeForName[0], keyForName[0][0]) #
                P3 = (timeForName[0] + keyForName[0][1][1], keyForName[0][0] + keyForName[0][1][2])

                t = (timeInKeyframes - self.firstTime) / (timeForName[0] - self.firstTime)

            else :
                
                if timeInKeyframes >= timeForName[-1]:
                    continue

                i = 0
                while timeInKeyframes > timeForName[i]:
                    i += 1

                i = i - 1
                
                P0 = (timeForName[i], keyForName[i][0])
                P1 = (timeForName[i] + keyForName[i][2][1], keyForName[i][0] + keyForName[i][2][2])
                P2 = (timeForName[i+1], keyForName[i+1][0]) #
                P3 = (timeForName[i+1] + keyForName[i+1][1][1], keyForName[i+1][0] + keyForName[i+1][1][2]) 

                t = (timeInKeyframes - timeForName[i]) / (timeForName[i+1] - timeForName[i])

            if t > 1:
                t = 1.0

            target_joints[name] = ((1-t) ** 3) * P0[1] + 3 * ((1-t) ** 2) * t * P1[1] + 3 * (1-t) * (t ** 2) * P2[1] + (t**3) * P3[1]
                

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
