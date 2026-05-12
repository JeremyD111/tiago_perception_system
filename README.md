\# TIAGo Perception System



Repositorio unificado para el procesamiento, compresión y concatenación de nubes de puntos del robot TIAGo y cámaras externas (Sputnik/Orbbec). Desarrollado para el IOC-UPC.



\## Estructura del Workspace

El repositorio funciona como un meta-workspace de ROS 2 que incluye:

\* `ros2\_tiago\_perception`: Nodos customizados (Filtrado con VAMP, Compresión, y Concatenación). Incluye la librería VAMP localmente.

\* `cloudini\_ros`: Sistema de descompresión.

\* `point\_cloud\_interfaces`: Definición de mensajes personalizados requeridos por Cloudini.



\## Instrucciones de Compilación (Ubuntu / ROS 2)

No es necesario instalar VAMP externamente, el compilador lo armará automáticamente.



```bash

\# 1. Clonar el repositorio

git clone <URL\_DEL\_REPOSITORIO> tiago\_perception\_system

cd tiago\_perception\_system



\# 2. Compilar

colcon build --symlink-install



\# 3. Cargar el entorno

source install/setup.bash

Ejecución de Nodos

1\. Filtrado y Compresión en el Origen (Sputnik / Orbbec)

Filtra la nube externa usando VAMP y la comprime usando Cloudini para enviarla por la red.



Bash

ros2 run ros2\_tiago\_perception vamp\_cloudini\_node

2\. Descompresión en el Destino (TIAGo)

Recibe la nube comprimida por la red y la restaura a PointCloud2.



Bash

ros2 run cloudini\_ros topic\_converter

3\. Filtrado Interno (TIAGo)

Aplica el filtro VAMP a las nubes propias del TIAGo para descartar colisiones con sus propios brazos.



Bash

ros2 run ros2\_tiago\_perception vamp\_filter\_node2

4\. Concatenación

Unifica las múltiples nubes procesadas en un solo tópico.



Bash

ros2 run ros2\_tiago\_perception pointcloud\_concatenate\_node



\---



\### Pasos finales para subirlo a GitHub:



1\. Ve a tu cuenta de GitHub en el navegador y crea un repositorio nuevo y vacío (no le agregues README ni licencia desde la página).

2\. Guarda el archivo `README.md` que acabas de crear en tu carpeta.

3\. En tu terminal Git Bash, ejecuta estos comandos (reemplazando la URL por la de tu nuevo repositorio):



```bash

git add README.md

git commit -m "docs: add README with compilation and execution instructions"

git branch -M main

git remote add origin https://github.com/TU\_USUARIO/TU\_REPOSITORIO.git

git push -u origin main

```

