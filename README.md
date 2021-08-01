# GVRP using Greedy Tabu Search

## Sobre makefile
Para compilar todos los archivos se debe ejecutar el comando
```bash
	make 
```

Para correr el ejecutable se debe utilizar el siguiente comando:
```bash
./GVRPGreedyTabu AB{0}
```
donde {0} corresponde al número de la instancia. Por ejemplo, para la instancia AB101 se tiene:
```bash
./GVRPGreedyTabu AB101

```

Para eliminar los archivos compilados se debe ejecutar el comando:
```bash
	make clean
```

## Consideraciones
### Directorios
Para correcto funcionamiento, las instancias del problema deben ubicarse en `./instances/`.

Se puede cambiar el directorio de instancias cambiando el valor de la variable
```cpp
#define TARGET_DIRECTORY "./instances/"
```
en el archivo `instance_util.h`.

Las soluciones serán guardadas en la carpeta `./solutions/`. 
Se puede cambiar el directorio de soluciones cambiando el valor de la variable
```cpp
#define OUTPUT_DIRECTORY "./solutions/"
```
en el archivo `solver.h`.
### Librerias
No se requieren librerias externas.

### Sobre algoritmo
Se aplica Greedy Search para generar la solución inicial, que será mejorada a través de Tabu Search. Se aplican restricciones duras a la búsqueda Greedy, con tal de generar siempre el tour factible (ciclo que cumple con distancias de viaje y tiempo total de servicio de vehículo). TS como movimiento utiliza la heurística 2opt. No existe limite de vehículos en la flota.

## Experimientos
### runall.py
Corresponde a un script que permite correr todas las instancias (AB1 y AB2) con la configuración predeterminada.
### experiments.py
Corresponde a un script que realiza 480 pruebas para las distintas configuraciones para duplas (largo de lista tabu, cantidad de iteraciones), donde
el largo de la lista tabu puede tomar los valores de conjunto `L={3, 8, 13, 21}`, mientras que el numero de iteraciones pertenece al conjunto `I={8, 34, 55}`. Los experimientos corresponden a las pruebas para las duplas obtenidas al hacer `L x I` para las 40 instancias.

Es decir, por ejemplo se probará la instancia AB101 para las configuraciones `(3,8), (3,34), (3,55), (8,8)....`.
### tabuGreedy.ipynb
Corresponde a un Jupyter Notebook que representa los datos de pruebas de manera gráfica o en tablas. Se elaboran tablas de porcentajes de mejora de cada configuracion con el fin de determinar la mejor configuración promedio para cada grupo de instancias. Se definen tres grupos de instancias - de 50, 75 y de 100 clientes. Finalmente, se tabulan los datos de soluciones para las mejores configuraciones de cada grupo, indicando el número de clientes visitados, número de vehiculos enviados, calidad Greedy, calidad Tabu, entre otros parámetros.



