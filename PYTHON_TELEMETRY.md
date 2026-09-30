# Panel de telemetría en Python para Spyder

`tools/telemetry_dashboard.py` recibe la trama binaria del firmware, muestra
las 51 señales en tiempo real y permite grabar o exportar los datos a CSV. La
interfaz usa `tkinter` y gráficas propias, por lo que solo necesita `pyserial`.

## Instalación en Spyder

Abra la consola IPython que usa Spyder y ejecute:

```python
%pip install pyserial
```

Reinicie el kernel de Spyder después de instalarlo. No es necesario instalar
Matplotlib, NumPy, pandas ni PyQt para usar esta aplicación.

## Ejecución

1. Abra `tools/telemetry_dashboard.py` en Spyder.
2. Ejecute el archivo completo con **Run file**.
3. Presione **↻** para actualizar los puertos.
4. Seleccione el puerto virtual COM de la LaunchPad.
5. Conserve `460800` baud y presione **Conectar**.

El puerto usa 8 bits, sin paridad, un bit de parada y sin control de flujo. Si
CCS, PuTTY u otra aplicación tiene abierto el mismo COM, ciérrelo antes de
conectar desde Python.

La opción **Modo demo** genera señales simuladas a 100 Hz. Sirve para revisar
la interfaz, las gráficas y la exportación CSV sin conectar el microcontrolador.

## Contenido de la interfaz

- La vista inicial **Vista general** concentra enlace, campo magnético, actitud,
  motores y gráficas de control.
- **Sensores · 9 ejes** muestra acelerómetro, giroscopio y LIS2MDL en µT.
- Estado del enlace y edad de la última trama.
- Frecuencia real recibida y magnitud del campo magnético.
- Conteo de saltos de secuencia, huecos de timestamp y resincronizaciones.
- Indicador de actitud para roll y pitch.
- Vista superior de los cuatro motores con su pulso en microsegundos.
- Gráfica de heading magnético, referencia capturada y error de rumbo.
- Gráficas de aceleración horizontal compensada, velocidad y desplazamiento
  locales estimados, y correcciones angulares del freno inercial.
- Gráficas de actitud, giroscopio IMU directo, tasas angulares filtradas,
  errores PID, salidas PID, motores, aceleración y términos P/I/D por eje.
- Tabla con cabecera y las 51 variables enviadas por el firmware.
- Diagnóstico de bytes, tramas, descartes y estado de la grabación.

## Grabación CSV

**Grabar CSV** solicita un nombre de archivo y empieza a escribir cada trama
recibida. Presione el mismo botón para cerrar el archivo correctamente.

**Exportar historial** guarda las muestras que permanecen en la memoria gráfica,
incluso si no se inició una grabación previamente. El historial contiene hasta
12000 muestras, unos 120 segundos a 100 Hz.

El CSV usa UTF-8, separador coma y punto decimal. Sus columnas son:

1. Hora ISO del PC.
2. Tiempo transcurrido desde la conexión.
3. `sync`, versión, secuencia y timestamp del microcontrolador.
4. Las 51 señales en el mismo orden de `TELEMETRY.md`, incluidos los seis
   valores IMU, los tres ejes magnéticos, las tres variables de heading y las
   ocho señales del freno inercial horizontal.

## Uso para análisis inercial

La IMU produce datos a 416 Hz, mientras que la telemetría guarda el último
sample disponible a 100 Hz. Por ello, para integrar las filas del CSV use la
diferencia entre valores consecutivos de `timestamp_us`. No use `imu_dt_s`
como separación entre filas: ese campo describe el periodo interno de la
muestra IMU, aproximadamente 2.4 ms.

`accel_x/y/z` contiene fuerza específica e incluye gravedad. Antes de estimar
velocidad o posición es necesario estimar orientación con el giroscopio,
rotar la aceleración al marco terrestre, retirar gravedad y bias, filtrar y
solo entonces integrar. Integrar directamente las columnas del acelerómetro
produce deriva rápidamente, incluso con el dron inmóvil.

`mag_x/y/z` son las componentes calibradas en el marco FRD del dron: X hacia la
nariz, Y hacia la derecha y Z hacia abajo. El firmware ya retira el offset
hard-iron y aplica la matriz soft-iron y de orientación. Se repiten normalmente
en dos filas porque el magnetómetro trabaja a 50 Hz y la telemetría a 100 Hz.

`heading` es el rumbo magnético compensado por roll/pitch y filtrado. Al soltar
el stick de yaw, `heading_setpoint` captura el rumbo presente y
`heading_error` muestra la diferencia angular más corta en el intervalo
-180...180°. El lazo externo convierte este error en `yaw_rate_setpoint`; las
columnas de yaw del PID permiten evaluar la respuesta del lazo interno.

`horizontal_accel_x/y` es la aceleración horizontal compensada y filtrada que
usa el freno inercial. `horizontal_velocity_x/y` y
`horizontal_displacement_x/y` son integrales locales, con fuga y límites para
evitar que el bias crezca sin control. `drift_roll_correction` y
`drift_pitch_correction` son los grados añadidos a las consignas angulares
cuando los sticks están centrados. X positivo apunta hacia la nariz; Y positivo
representa movimiento hacia la izquierda según la orientación validada.

Estas variables permiten estudiar y frenar un movimiento transitorio. No son
posición absoluta: una IMU no puede distinguir indefinidamente entre reposo y
velocidad horizontal constante. Al mover cualquier stick, perder heading,
bajar throttle o completar el asentamiento, el firmware reinicia el origen
local de la estimación.

## Validación del decodificador

Desde la carpeta `tools`, ejecute:

```text
python -m unittest test_telemetry_dashboard.py -v
```

Las pruebas verifican tamaño y orden de la trama, lecturas parciales,
resincronización después de ruido, rechazo de una versión incorrecta y formato
del CSV.

## Interpretación de los contadores

- **Saltos de sequence:** el PC perdió una trama que el firmware sí construyó,
  o se perdieron bytes suficientes para requerir resincronización.
- **Huecos de timestamp:** separación mayor de 15 ms entre snapshots. También
  detecta periodos que el firmware omitió antes de construir una trama.
- **Bytes descartados:** datos anteriores a la palabra de sincronía.
- **Candidatos inválidos:** sync encontrada con versión distinta de 6 o valores
  flotantes no finitos.
- **Overflow cola PC:** la interfaz no consumió eventos tan rápido como llegaron.

La trama no incluye CRC. La aplicación comprueba sync, versión y valores
finitos, pero un byte alterado que todavía produzca un `float32` finito puede
pasar inadvertido. Para análisis de control, confirme también continuidad de
secuencia y timestamp.
