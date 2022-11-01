# DFM27URO135ML

Este repositorio consiste en el intento de desarrollo de un paquete de Ros que permita reconocer, configurar y adquirir datos de una camara [The Imaging Source DFM 27UR0135-ML](https://www.theimagingsource.com/products/board-cameras/usb-3.0-color/dfm27ur0135ml/) empleando C++.

##### MENU

- [Hardware](#hardware)
- [Instalación](#instalacion)
- [Configuración](#configuracion)
- [Implementación](#implementacion)

<a name="hardware"/>

## 1. Hardware

- CÁMARA DE PLACA USB 3.0 RGB: [The Imaging Source DFM 27UR0135-ML](https://www.theimagingsource.com/products/board-cameras/usb-3.0-color/dfm27ur0135ml/)
- Sensor CMOS de semiconductores de 1/3 de pulgada (AR0135)
- 1280×960 (1,2 MP), hasta 60 fps
- Obturador global
- Entradas de disparador y E/S
- Fabricado por The Imaging Source

<a name="instalacion"/>

## 2. Instalación

[TIS Camera-0.14.0](https://github.com/TheImagingSource/tiscamera/releases):

Descargue el archivo "source code (zip)" de la versión 0.14.0 de tiscamera del siguiente [link](https://github.com/TheImagingSource/tiscamera/releases) y luego realice los siguientes pasos:

```console
$ cd tiscamera-v-tiscamera-0.14.0
$ ./scripts/dependency-manager install
$ mkdir build
$ cd build
$ cmake -DBUILD_ARAVIS=OFF ..
$ make -j
$ sudo make install
```

Clone este repositorio en ~/catkin_ws/src

```console
$ git clone https://github.com/JohanAvilaU17/DFM27URO135ML.git
$ cd ..
$ catkin_make
```

<a name="configuracion"/>

## 3. Configuración

Para realizar cambios en las propiedades o en el formato de las cámaras, modifiqué el siguiente archivo de configuración respetando la estructura YAML:

- params.yaml:

  ```console
  dfm27uro135ml:
    serial:
      cam: "13910330"

    properties:
      GPIn: 0
      GPOut: 0
      OffsetAutoCenter: true
      OffsetX: 0
      OffsetY: 0
      OverrideScanningMode: 0
      StrobeEnable: false
      StrobeExposure: false
      StrobePolarity: false
      TriggerDelayUs: 0
      TriggerGlobalResetRelease: false
      TriggerMode: false
      cameraWhitebalance: false
      whitebalanceAuto: true
      whitebalanceBlue: 64
      whitebalanceGreen: 64
      whitebalanceModuleEnabled: true
      whitebalanceRed: 64
      Brightness: 16
      BrightnessReference: 60
      ExposureTimeUs: 33300
      ExposureAuto: true
      ExposureMin: 100
      ExposureMax: 33300
      Gain: 48
      GainDB100: 0
      GainAuto: false
      GainMin: 48.0
      GainMax: 2047.0
      ExposureROILeft: 0
      ExposureROIWidth: 1280
      ExposureROITop: 0
      ExposureROIHeight: 960

    formats:
      name: "video/x-raw"
      format: "BGR"
      width: 1024
      height: 768
      fps: 10
  ```

  Con base en la informacion suministrada por los siguientes comandos :

- Lista los dispositivos conectados :

  ```console
  $ rosrun dfm27uro135ml dfm27uro135ml_ListDevices
  ```

  Example output :

  ```console
  -> Model: DFM 27UR0135-ML Serial: 26810384 Identifier: /dev/video0 Type: v4l2
  ```

- Lista las propiedades de la cámara :

  ```console
  $ rosrun dfm27uro135ml dfm27uro135ml_ListProperties
  ```

  Example output :

  ```console
  -> Model: DFM 27UR0135-ML Serial: 26810384 Identifier: /dev/video0 Type: v4l2

  Properties before state PLAYING:
  whitebalance-red(integer) min: 0 max: 255 step: 1 value: 64 default: 64  grouping Color Whitebalance
  whitebalance-green(integer) min: 0 max: 255 step: 1 value: 64 default: 64  grouping Color Whitebalance
  whitebalance-blue(integer) min: 0 max: 255 step: 1 value: 64 default: 64  grouping Color Whitebalance
  whitebalance-auto(boolean) value: true default: true  grouping Color Whitebalance
  whitebalance-module-enabled(boolean) value: true default: true  grouping Color Whitebalance
  camera-whitebalance(boolean) value: false default: false  grouping Color Whitebalance
  Brightness Reference(integer) min: 0 max: 255 step: 1 value: 128 default: 128  grouping Exposure Exposure
  Exposure Auto(boolean) value: true default: true  grouping Exposure Exposure
  Exposure Min(integer) min: 100 max: 2147483647 step: 1 value: 100 default: 100  grouping Exposure Exposure
  Exposure Max(integer) min: 100 max: 2147483647 step: 1 value: 16666 default: 2147483647  grouping Exposure Exposure
  Gain Auto(boolean) value: true default: true  grouping Exposure Gain
  Gain Min(double) min: 48.000000 max: 2047.000000 step: 1.000000 value: 48.000000 default: 48.000000  grouping Exposure Gain
  Gain Max(double) min: 48.000000 max: 2047.000000 step: 1.000000 value: 2047.000000 default: 2047.000000  grouping Exposure Gain
  Exposure ROI Left(integer) min: 0 max: 1271 step: 1 value: 0 default: 0  grouping Exposure ROI
  Exposure ROI Width(integer) min: 8 max: 1280 step: 1 value: 1280 default: 1280  grouping Exposure ROI
  Exposure ROI Top(integer) min: 0 max: 951 step: 1 value: 0 default: 0  grouping Exposure ROI
  Exposure ROI Height(integer) min: 8 max: 960 step: 1 value: 960 default: 960  grouping Exposure ROI
  Brightness(integer) min: 0 max: 4095 step: 1 value: 16 default: 16  grouping Exposure Brightness
  Gain(integer) min: 48 max: 2047 step: 1 value: 32 default: 48  grouping Exposure Gain
  Exposure Time (us)(integer) min: 100 max: 1000000 step: 1 value: 1000 default: 33333  grouping Exposure Exposure
  Gain (dB/100)(integer) min: 0 max: 1629 step: 1 value: 0 default: 0  grouping Unknown INVALID_PORPERTY
  Trigger Mode(boolean) value: false default: false  grouping Special Trigger Mode
  Trigger Delay (us)(integer) min: 0 max: 10000000 step: 10 value: 150 default: 0  grouping Special Trigger Mode
  Strobe Enable(boolean) value: false default: false  grouping Special Strobe Enable
  Strobe Polarity(boolean) value: false default: false  grouping Special Strobe Enable
  Strobe Exposure(boolean) value: false default: false  grouping Special Strobe Enable
  GPOut(integer) min: 0 max: 1 step: 1 value: 0 default: 0  grouping Special GPIO
  GPIn(integer) min: 0 max: 1 step: 1 value: 0 default: 0  grouping Special GPIO
  Offset X(integer) min: 0 max: 1184 step: 2 value: 0 default: 0  grouping Partial Scan Offset Auto Center
  Offset Y(integer) min: 0 max: 864 step: 2 value: 0 default: 0  grouping Partial Scan Offset Auto Center
  Offset Auto Center(boolean) value: true default: true  grouping Partial Scan Offset Auto Center
  Override Scanning Mode(integer) min: 0 max: 2 step: 1 value: 0 default: 0  grouping Partial Scan Override Scanning Mode
  Trigger Global Reset Release(boolean) value: false default: false  grouping Special Trigger Mode
  ```

- Lista los formatos de la cámara:

  ```console
  $ rosrun dfm27uro135ml dfm27uro135ml_ListFormats
  ```

  Example output :

  ```console
  -> Model: DFM 27UR0135-ML Serial: 26810384 Identifier: /dev/video0 Type: v4l2

  video/x-bayer grbg - 1280 X 960 - 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-bayer grbg - 1280 X 720 - 80/1 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-bayer grbg - 1024 X 768 - 80/1 70/1 60/1 40/1 30/1 20/1 10/1
  video/x-bayer grbg - 640 X 480 - 120/1 90/1 60/1 30/1 15/1 10/1
  video/x-raw GRAY8 - 1280 X 960 - 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-raw GRAY8 - 1280 X 720 - 80/1 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-raw GRAY8 - 1024 X 768 - 80/1 70/1 60/1 40/1 30/1 20/1 10/1
  video/x-raw GRAY8 - 640 X 480 - 120/1 90/1 60/1 30/1 15/1 10/1
  video/x-raw GRAY16_LE - 1280 X 960 - 70/1 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-raw GRAY16_LE - 640 X 480 - 120/1 90/1 60/1 30/1 15/1 10/1
  video/x-raw { RGBx xRGB BGRx xBGR RGBA ARGB BGRA ABGR } - 1280 X 960 - 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-raw { RGBx xRGB BGRx xBGR RGBA ARGB BGRA ABGR } - 1280 X 720 - 80/1 60/1 50/1 40/1 30/1 20/1 10/1
  video/x-raw { RGBx xRGB BGRx xBGR RGBA ARGB BGRA ABGR } - 1024 X 768 - 80/1 70/1 60/1 40/1 30/1 20/1 10/1
  video/x-raw { RGBx xRGB BGRx xBGR RGBA ARGB BGRA ABGR } - 640 X 480 - 120/1 90/1 60/1 30/1 15/1 10/1
  ```

  <a name="implementacion"/>

## 4. Implementación

- Prueba de vídeo:

  ```console
  $ source devel/setup.bash
  $ roslaunch dfm27uro135ml LiveStreamCam.launch
  ```

- Publicar las imágenes en Ros con el tópico :

  - 'dfm27uro135ml/cam/image_raw'

  ```console
  $ source devel/setup.bash
  $ roslaunch dfm27uro135ml PublisherImg.launch
  ```
