# Panel de telemetría en Python para Spyder

`tools/telemetry_dashboard.py` recibe la trama binaria del firmware, muestra
las 37 señales en tiempo real y permite grabar o exportar los datos a CSV. La
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

- La vista inicial **IMU cruda · 6 ejes** muestra simultáneamente acelerómetro
  en m/s² y giroscopio en rad/s. El título incluye **RAW IMU · Schema 2** para
  distinguir esta versión de una ventana anterior que haya quedado abierta.
- Estado del enlace y edad de la última trama.
- Frecuencia real recibida y secuencia del firmware.
- Conteo de saltos de secuencia, huecos de timestamp y resincronizaciones.
- Indicador de actitud para roll y pitch.
- Vista superior de los cuatro motores con su pulso en microsegundos.
- Gráficas de actitud, giroscopio IMU directo, tasas angulares filtradas,
  errores PID, salidas PID, motores, aceleración y términos P/I/D por eje.
- Tabla con cabecera y las 37 variables enviadas por el firmware.
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
4. Las 37 señales en el mismo orden de `TELEMETRY.md`, incluidos los seis
   valores IMU sin filtrado en unidades físicas.

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
- **Candidatos inválidos:** sync encontrada con versión distinta de 2 o valores
  flotantes no finitos.
- **Overflow cola PC:** la interfaz no consumió eventos tan rápido como llegaron.

La trama no incluye CRC. La aplicación comprueba sync, versión y valores
finitos, pero un byte alterado que todavía produzca un `float32` finito puede
pasar inadvertido. Para análisis de control, confirme también continuidad de
secuencia y timestamp.
