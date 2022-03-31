
        B = np.matrix([
                    [-1.0, 1.0, 1.0, -1.0],
                    [1.0, -1.0, 1.0, -1.0],
                    [1.0, 1.0, -1.0, -1.0],
                    [1.0, 1.0, 1.0, 1.0]])

        P = np.linalg.pinv(B)
        # quad x
        P = np.matrix([
            [ -0.707107,  0.707107,  1.000000,  1.000000 ],
            [  0.707107, -0.707107,  1.000000,  1.000000 ],
            [  0.707107,  0.707107, -1.000000,  1.000000 ],
            [ -0.707107, -0.707107, -1.000000,  1.000000 ]])
        B = np.linalg.pinv(P)

        count = 0
        while(count<10):
            count = count + 1
            print("AccP: ", drone.master.attitude.roll)
            print("AccP: ", drone.master.attitude.pitch)
            print("AccP: ", drone.master.attitude.yaw)

            accR = drone.master.attitude.roll
            accP = drone.master.attitude.pitch
            accY = drone.master.attitude.yaw

            p_dot_sp = accR # roll acceleration (p is the roll rate)
            q_dot_sp = accP # pitch acceleration
            r_dot_sp = accY # yaw acceleration
            T_sp = 1.0 # vertical thrust
            m_sp = np.matrix([p_dot_sp, q_dot_sp, r_dot_sp, T_sp]).T # Vector of desired "accelerations"

            # Actuators output saturations
            u_max = 1.0
            u_min = 0.0
            
            (u, u_new) = normal_mode(m_sp, P, u_min, u_max)

            # Saturate the outputs between 0 and 1
            u_new_sat = np.maximum(u_new, np.matlib.zeros(u.size).T)
            u_new_sat = np.minimum(u_new_sat, np.matlib.ones(u.size).T)

            # Display some results
            print("u_new = {}\n".format(u_new))
            # print("u_new_sat = {}\n".format(u_new_sat))
            print("Desired accelerations = {}\n".format(m_sp))
            # Compute back the allocated accelerations
            m_new = B * u_new_sat

            send_ned_velocity(u_new_sat[0],u_new_sat[1],u_new_sat[2],1)
            time.sleep(0.01)
