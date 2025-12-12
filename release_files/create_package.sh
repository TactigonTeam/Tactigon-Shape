docker build -t tactigon-shapes .
#salva e comprimi il file tar con l'immagine
$ docker save tactigon-shapes | gzip > tactigon-shapes.tar.gz

#questo comando deve essere lanciato sul pc del ricevente
# gunzip -c tactigon_shape.tar.gz | docker load
