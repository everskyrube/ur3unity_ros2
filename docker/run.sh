# This allows access to the linux graphical user interface X11
sudo xhost +
# Compose the containerdocker compose -f docker-compose.yaml up
# --build / Rebuilds images before starting containers.
docker compose -f docker-compose.yml up -d