# ros2custom

**ros2custom** es un plugin que permite ejecutar varios comandos de la librería ros2cli al mismo tiempo en un terminal y representarlos de una forma clara.

## Instalación

Para la instalación lo trataremos como cualquier otro paquete

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
  ros2 custom monitor -h
   ```
      
## Utilización

Para utilizarlo, deberemos ejecutar el comando `ros2 custom monitor` y pasarle un topic al que queramos referirnos:
    ```terminal
    #Ejemplo
    ros2 custom monitor /image
    ```
Se mostrará por el terminal una tabla: 
```bash
      ╒═══════╤════════╤═════════╤════════════════╕
      │       │ HZ     │ Delay   │ BW             │
      ╞═══════╪════════╪═════════╪════════════════╡
      │ Valor │ - hz   │ - s     │ - MB/s         │
      ├───────┼────────┼─────────┼────────────────┤
      │ Info  │ win: - │ win: -  │ msg_size: - MB │
      ╘═══════╧════════╧═════════╧════════════════╛
  ```

Donde se irán escribiendo los diferentes datos que se iran capturando. 

### Argumentos:

  También podremos configurar una serie de argumentos para diferentes funcionalidades 

- `-w`: permite cambiar el tamaño de la ventana de la frecuencia de publicación, el delay y el ancho de banda
- `-qos`: muestra una lista con los nodos que publican y reciben del topic y su configuración de qos (similar a `ros2 topic info -v`)
- `-net`: muestra una tabla con la velocidad de envío y recepción de cada una de las interfaces de red
  ```bash
      [Network Info]
      ╒════════════╤══════╤══════╕
      │ Interfaz   │ RX   │ TX   │
      ╞════════════╪══════╪══════╡
      ╘════════════╧══════╧══════╛
  ```



      