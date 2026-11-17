#!/bin/bash

# ------------------------------------------------------------
#   RUN CONTAINER
# ------------------------------------------------------------

# Abilita i permessi per accedere al server X (GUI / RViz)
xhost +local:docker

# Crea la cartella della cronologia bash (evita che Docker la crei come root)
mkdir -p docker/history

# Assicura che il container sia avviato in background se non è attivo
docker compose up -d

# Entra nel container esistente usando sempre lo stesso (collegamento interattivo -it)
docker exec -it flexiv bash

# Disabilita i permessi di accesso al server X alla chiusura del terminale (opzionale)
xhost -local:docker
