# ros2custom

**ros2custom** es un plugin que permite ejecutar varios comandos nativos de ROS al mismo tiempo en un terminal y representarlos de una forma clara.

## Instalación

Para la instalación lo trataremos como cualquier otro paquete de ROS

1. **Clonar el repositorio**
```terminal
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rmorales-catec/ros2custom.git
```

2. **Compilar el workspace y cargarlo**
  ```terminal
  cd ~/ros2_ws
  colcon build --packages-select ros2custom
  source install/setup.zsh
  ```

  Para comprobar que la instalación se realizó correctamente y que podemos utilizarlo:
  ```terminal
  ros2 custom topic -h
  ros2 custom node -h
   ```
      
## Utilización

### Verb `topic`

Para utilizarlo, deberemos ejecutar el comando `ros2 custom topic` y pasarle un topic al que queramos referirnos:
  ```terminal
  #Ejemplo
  ros2 custom topic /turtle1/pose
  ```
Se mostrará por el terminal una tabla que contiene los datos de la frecuencia de publicación, el delay y el ancho de banda consumido, así como el tamaño de la ventana y el tamaño medio de los mensajes: 
```bash
    ╒═══════╤════════╤═════════╤════════════════╕
    │       │ HZ     │ Delay   │ BW             │
    ╞═══════╪════════╪═════════╪════════════════╡
    │ Valor │ - hz   │ - s     │ - MB/s         │
    ├───────┼────────┼─────────┼────────────────┤
    │ Info  │ win: - │ win: -  │ msg_size: - MB │
    ╘═══════╧════════╧═════════╧════════════════╛
  ```

#### Argumentos:

  También podremos configurar una serie de argumentos para diferentes funcionalidades:

- `-w`: permite cambiar el tamaño de la ventana de la frecuencia de publicación, el delay y el ancho de banda.
- `-qos`: muestra una lista con los nodos que publican y reciben del topic y su configuración de qos (similar a `ros2 topic info -v`)
- `-net`: muestra una tabla con el tráfico entrante y saliente de cada una de las interfaces de red.
  ```bash
    [Network Info]
    ╒════════════╤══════╤══════╕
    │ Interfaz   │ RX   │ TX   │
    ╞════════════╪══════╪══════╡
    ╘════════════╧══════╧══════╛
  ```

  Toda la información que aparece se irá refrescando cada segundo, permitiendo monitorizar todo en tiempo real


### Verb `node`

Para utilizarlo, deberemos ejecutar el comando `ros2 custom node` y pasarle el nombre de un nodo: 
  ```terminal
  #Ejemplo
  ros2 custom node /turtlesim
  ```
Se mostrará una tabla con el nombre y el valor de todos los parámetros configurables que tenga ese nodo:
  ```bash
    [Parámetros actualizados]
    ╒════════════════╤═══════════╕
    │ Nombre         │ Valor     │
    ╞════════════════╪═══════════╡
    ╘════════════════╧═══════════╛

  ```

#### Argumentos:

  También podremos configurar una serie de argumentos para diferentes funcionalidades:
- `-serv`: muestra una tabla con los services que tiene disponibles ese nodo y el tipo de mensaje que utiliza.
    ```bash
      [Servicios disponibles]
    ╒════════════════╤═══════════╕
    │ Nombre         │ Tipo(s)   │
    ╞════════════════╪═══════════╡
    ╘════════════════╧═══════════╛
    ```
- `-act`: muestra una lista con las acciones que tiene disponibles el nodo (similar a `ros2 action list`)
- `-pubsub`: muestra una tabla con los topics a los que está suscrito y en los que publica el nodo.
    ```bash
      [Topics]
    ╒════════════════╤══════════════╕
    │ Susciptores    │ Publishers   │
    ╞════════════════╪══════════════╡
    ╘════════════════╧══════════════╛
    ```
