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
        self.startTime = 0 
        self.endTime = 0 

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        (names, times, keys) = keyframes
        self.currentTime = self.perception.time
        timeInKeyframes = self.currentTime - self.startTime

        if self.endTime == 0:
            for valueIndex in range(0, len(names)):
                name = names[valueIndex]
                timesForName = times[valueIndex]
                keysForName = keys[valueIndex]
                if timesForName[len(timesForName)-1]> self.endTime:
                   self.endTime = timesForName[len(timesForName)-1]


        if self.endTime < timeInKeyframes or self.startTime == 0:
            self.startTime = self.currentTime
            timeInKeyframes = self.currentTime - self.startTime


        for valueIndex in range(0, len(names)):
            name = names[valueIndex]
            timesForName = times[valueIndex]
            keysForName = keys[valueIndex]

            if timeInKeyframes < timesForName[0]:
                
                if not name in self.perception.joint:
                    continue

                P_0 = (timeInKeyframes, self.perception.joint[name])
                P_1 = (timesForName[0] + keysForName[0][1][1], keysForName[0][0] + keysForName[0][1][2])
                P_2 = (timesForName[0], keysForName[0][0]) #
                P_3 = (timesForName[0] + keysForName[0][1][1], keysForName[0][0] + keysForName[0][1][2])

                t = (timeInKeyframes - self.startTime) / (timesForName[0] - self.startTime)

               
            else:

                if timeInKeyframes >= timesForName[-1]:
                    continue

                i = 0
                while timeInKeyframes > timesForName[i]:
                    i += 1

                i = i - 1

                P_0 = (timesForName[i], keysForName[i][0])
                P_1 = (timesForName[i] + keysForName[i][2][1], keysForName[i][0] + keysForName[i][2][2])
                P_2 = (timesForName[i+1], keysForName[i+1][0]) #
                P_3 = (timesForName[i+1] + keysForName[i+1][1][1], keysForName[i+1][0] + keysForName[i+1][1][2]) 

                t = (timeInKeyframes - timesForName[i]) / (timesForName[i+1] - timesForName[i])
                
                
            if t > 1:
                t = 1.0
            elif t < 0:
                t = 0.0

            target_joints[name] = ((1-t) ** 3) * P_0[1] + 3 * ((1-t) ** 2) * t * P_1[1] + 3 * (1-t) * (t ** 2) * P_2[1] + (t**3) * P_3[1]
    

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
