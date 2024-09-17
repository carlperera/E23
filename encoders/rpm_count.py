"""1---calculate motor counts"""
            # calculate motor speed:
            # revs of the motor shaft:
            motor1_counts = robot.motor1_encoder.steps
            motor1_revs = motor1_counts/ (CPR * GEAR_RATIO)
            # convert revolutions to rpm:
            motor1_rpms = (motor1_revs / time_interval) * 60
            print(f"1----RPM = {motor1_rpms:.2f} RPM, count = {motor1_counts: .2f}")
        
            motor2_counts = robot.motor2_encoder.steps
            motor2_revs = motor2_counts/ (CPR * GEAR_RATIO)
            # convert revolutions to rpm:
            motor2_rpms = (motor2_revs / time_interval) * 60
            print(f"2----RPM = {motor2_rpms:.2f} RPM, count = {motor2_counts: .2f}")
    

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            motor1_counts = 0
            motor2_counts = 0


            time.sleep(0.1)
                

        except KeyboardInterrupt:

            
            print("Done")
            break