import rospy, argparse
import os, sys, time, subprocess
import numpy as np
from gazebo_msgs.srv import DeleteModel, SpawnModel
import random
# subprocess.run(["ls", "-l"])
PI = 3.14
class Creator:
    def __init__(self, **kwargs):
        global PI
        l = 0.32
        d = 0.16
        h = d * np.sin(45./180.*3.14)
        angle = PI * 45. / 180.


        self.HL_limits = {
            'H': {'min': 0.5 * np.sin(angle) * d, 'max': np.sin(angle) * d*0.8},
            # 'H': {'min': 10./17.2*h, 'max': h},
            # 'L': {'min': np.cos(angle) * l, 'max': l} #
        }

        a1 = 20.
        a2 = 30.

        self.amax = np.tan(a1 / 180.0 * 3.14)  # 0.2678069474078098
        self.amin = np.tan(a2 / 180.0 * 3.14)  # 0.5769964003928729
        amid = (self.amax + self.amin) / 2.

        H = self.HL_limits['H']['max']
        Hh = (self.HL_limits['H']['max'] + self.HL_limits['H']['min']) * 0.5
        h = self.HL_limits['H']['min']

        self.HL_test = {
            'small': {
                'H': h,
                'L': h / self.amax
            },
            'medium': {
                'H': Hh,
                'L': Hh / amid

            },
            'big': {
                'H': H,
                'L': H / self.amin

            }
        }

        file_path = os.path.dirname(os.path.realpath(__file__))
        path = '/'.join(file_path.split('/')[:-3])
        path = os.path.join(path, 'worlds')
        self.template_file = os.path.join(path, 'stairs_template.sdf')
        self.launch_file = os.path.join(path, 'stairs_launch.sdf')

    def create_random_stair(self):
        kwargs = self.generate_parameters()
        self.made_model(**kwargs)
        return kwargs

    def made_model(self, **params):
        H = params['H']
        L = params['L']
        f = open(self.template_file, 'r')
        lines = f.readlines()
        f.close()
        for l, line in enumerate(lines):
            splitted = line.split('&')
            if len(splitted) > 1:
                for i in range(1, len(splitted), 2):
                    splitted[i] = str(round(eval(splitted[i]), 2))
                lines[l] = ' '.join(splitted)
        with open(self.launch_file, 'w') as f:
            for item in lines:
                f.write("%s\n" % item)

    def delete_model(self):
        delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete('stairs')
        time.sleep(0.2)

    def spawn_model(self):
        # file_path = '~/dev/catkin_ws/src/tracked_robot/worlds/stairs.sdf'
        subprocess.run('rosrun gazebo_ros spawn_model -file '+
                        self.launch_file+' -sdf -model stairs',
                       shell=True, check=True)
        time.sleep(0.2)

    def generate_parameters(self):
        kwargs = {'H': 0., 'L': 0.}
        # for k in kwargs.keys():
        #     kwargs[k] = self.HL_limits[k]['min']
        #     kwargs[k] += random.random()*(self.HL_limits[k]['max'] - self.HL_limits[k]['min'])

        kwargs['H'] = self.HL_limits['H']['min'] + \
                      (self.HL_limits['H']['max'] - self.HL_limits['H']['min']) \
                      * random.random()

        kwargs['L'] = kwargs['H'] / self.amin + \
             (kwargs['H'] / self.amax - kwargs['H'] / self.amin) * random.random()

        kwargs['slope'] = np.arctan(kwargs['H']/kwargs['L'])
        print('\nSTAIR SLOPE {}\n '.format(kwargs['slope']))
        return kwargs

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--delete", action='store_true', help="delete stair")
    parser.add_argument("-s", "--spawn", action='store_true', help="spawn stair")
    parser.add_argument("-r", "--respawn", action='store_true', help="respawn stair")
    parser.add_argument("-m", "--made", action='store_true', help="parse template and insert parameters")
    parser.add_argument("-b", "--build", nargs='+', type=float, help="build stair with certain parameters")
    parser.add_argument("-g", "--generate", action='store_true', help="spawn random stair")
    args = parser.parse_args()
    c = Creator()
    if args.delete:
        c.delete_model()
    elif args.spawn:
        c.spawn_model()
    elif args.made:
        c.delete_model()
        c.made_model(**{'H': 0.1, 'L': 0.4})
        c.spawn_model()
    elif args.respawn:
        c.delete_model()
        c.spawn_model()
    elif args.build:
        c.delete_model()
        c.made_model(**{'L': args.build[0], 'H': args.build[1]})
        c.spawn_model()
    elif args.generate:
        c.delete_model()
        kwargs = c.generate_parameters()
        c.made_model(**kwargs)
        c.spawn_model()