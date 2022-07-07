#!/bin/bash
if [[ "$EUID" -ne 0 ]]; then
  echo "Please run as 'sudo'"
  exit
fi



# Запоминаем папку запуска
export project_dir_path=`pwd`/

#export DISPLAY=:1
export DISPLAY=:0
xhost +


cd ${project_dir_path}



echo ""
echo ""
echo "=============================================="
echo "[--]    stage [docker build]"
echo "=============================================="
echo ""
sleep 1


cd ${project_dir_path}
# Поднимаем контейнеры из docker-compose.yml
docker-compose up -d --build # --project-name mashina

echo ""
echo ""
echo "=============================================="
echo "[OK]    stage [end]"
echo "BUILD END"
echo "=============================================="
cd ${project_dir_path} && chmod 777 -R *
