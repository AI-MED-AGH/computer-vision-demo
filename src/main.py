from supervisor import Supervisor
#test modelu chwytaka

def main():
    config = {
        "urdf_path": "../robot.urdf",
        "end_effector_frame_name": "Tip_Gripper_Servo",
        "use_meshcat": True
    }
    app = Supervisor(config)
    app.run()

if __name__ == "__main__":
    main()