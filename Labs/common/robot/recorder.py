#! /usr/bin/env python3


from robot_actions import *

TIMEOUT_DURATION = 20


def record(base_cyclic):
    cur_joint = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    joint_list = None
    xyz_list = None
    flag = True
    time_flag = False
    curr_err = 10
    tol = 1e-2
    timer = time.time()

    while flag == True:
        try:
            for i in range(len(base_cyclic.RefreshFeedback().actuators)):
                cur_joint[i] = base_cyclic.RefreshFeedback().actuators[i].position
                cur_end_xyz = np.array([base_cyclic.RefreshFeedback().base.tool_pose_x,
                                        base_cyclic.RefreshFeedback().base.tool_pose_y,
                                        base_cyclic.RefreshFeedback().base.tool_pose_z])
            if joint_list is None:
                joint_list = cur_joint
                xyz_list = np.array([base_cyclic.RefreshFeedback().base.tool_pose_x,
                                     base_cyclic.RefreshFeedback().base.tool_pose_y,
                                     base_cyclic.RefreshFeedback().base.tool_pose_z])

            else:
                joint_list = np.vstack((joint_list, cur_joint))
                xyz_list = np.vstack((xyz_list, cur_end_xyz))
                curr_err = np.linalg.norm((xyz_list[-1, :] - xyz_list[-2, :], xyz_list[-1, :] - xyz_list[-2, :]))
                # curr_err += np.linalg.norm((joint_list[-1, :] - joint_list[-2, :], joint_list[-1, :] - joint_list[-2, :]))

            print("Curr Gripper X {}, Y {}, Z {}".format(*cur_end_xyz))
            print("Curr Joints Q1 {}, Q2 {}, Q3 {}, Q4 {},  Q5 {} , Q6 {} \n To stop recording press Ctrl+C\n".format(
                *cur_joint))
            # print(curr_err < tol)
            # print(time.time() - timer)

            if (curr_err < tol) and (xyz_list.shape[0] > 10):
                if not time_flag:
                    time_flag = True
                    timer = time.time()

                if time.time() - timer > 10.:
                    return (joint_list, xyz_list)
            else:
                time_flag = False

        except KeyboardInterrupt:
            return (joint_list, xyz_list)


def save(data_to_save, prefix='4'):

    logdir_prefix = f'lab-0{prefix}'
    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), f'../../Lab{prefix}/data')

    if not (os.path.exists(data_path)):
        os.makedirs(data_path)

    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)

    print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")

    import pickle
    with open(logdir + '/data' + '.pkl', 'wb') as h:
        pickle.dump(data_to_save, h)


if __name__ == "__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
        import utilities

        # Parse arguments
        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)

            try:
                joint_trajectory = record(base_cyclic)
                save(joint_trajectory)
            except:
                print(e)
            finally:
                print('done')

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass
