import subprocess
import signal
import time
import os
import sys

def launch_ros2_node_with_profile():
    # Path al tuo file YAML
    profile_path = "cfg_test.yaml"

    # Comando da eseguire
    cmd = [
        "ros2", "run", "camera_ros", "camera_node",
        "--ros-args", "--params-file", profile_path
    ]

    # Avvia il nodo ROS2
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        preexec_fn=os.setsid  # crea un nuovo gruppo di processi per gestire SIGTERM facilmente
    )

    print(f"[INFO] Nodo camera_node avviato (PID: {process.pid})")
    return process


def terminate_process(process):
    if process and process.poll() is None:  # se il processo è ancora attivo
        print("[INFO] Arresto del nodo ROS2...")
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # termina l'intero gruppo
        process.wait(timeout=5)
        print("[INFO] Nodo terminato correttamente.")


if __name__ == "__main__":
    try:
        node_process = launch_ros2_node_with_profile()

        # Simuliamo runtime (es. 10 secondi)
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INFO] Interruzione da tastiera rilevata.")

    finally:
        terminate_process(node_process)
        print("[INFO] Uscita completata.")
        sys.exit(0)
