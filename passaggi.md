1. modificato immagini, pacchetti e percorsi vulcanexus da jazzy ad Humble nei docker file di cameratracking e shape 
2. rimposso pyaudio e pynput da requirements.txt
3. modificato percorso source dell'entrypoint da jazzy ad humble
4. installato vulcanexus humble seguendola guida a questo [link](https://docs.vulcanexus.org/en/latest/rst/installation/linux_binary_installation.html)
5. copiato il docker-compose (docker-compose.jetson.yml) ed ho modificato i networks in `network-mode: host` ed aggiunto `ipc: host` → per lanciare fare `docker compose -f docker-compose.jetson.yml up --build`
6. disabilitato l'uso della tabella raw in docker:
    - creo una cartella per le configurazioni personalizzate di docker:

    ```bash
    sudo mkdir -p /etc/systemd/system/docker.service.d
    ```

    - creo un file di configurazione per disabilitare la funzionalità:

    ```bash
    sudo tee /etc/systemd/system/docker.service.d/insecure_direct_routing.conf > /dev/null <<EOF
    [Service]
    Environment="DOCKER_INSECURE_NO_IPTABLES_RAW=1"
    EOF
    ```

    - ricarico systemd e riavvio docker:

    ```bash
    sudo systemctl daemon-reload
    sudo systemctl restart docker
    ```
7. in `tactigon_shapes/modules/shapes/extension.py`:
    - commentato l'import di `KeyboardController`: `# from pynput.keyboard import Controller as KeyboardController`
    - commentato `self.keyboard` nell'init della classe `ShapesApp`: `# self.keyboard = KeyboardController()`
8. in `tactigon_shapes/modules/shapes/static/js/custom_blocks.js`:
    - aggiunto `KeyboardController` in: `from tactigon_shapes.modules.shapes.extension import ShapesPostAction, LoggingQueue, KeyboardController`
    - commentato la funzione `keyboard_press` in `defineImportsAndLibraries`
