# Panel de telemetría en Python para Spyder

`tools/telemetry_dashboard.py` recibe la trama binaria del firmware, muestra
las 34 señales en tiempo real y permite grabar o exportar los datos a CSV. La
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

- Estado del enlace y edad de la última trama.
- Frecuencia real recibida y secuencia del firmware.
- Conteo de saltos de secuencia, huecos de timestamp y resincronizaciones.
- Indicador de actitud para roll y pitch.
- Vista superior de los cuatro motores con su pulso en microsegundos.
- Gráficas de actitud, tasas angulares, errores PID, salidas PID, motores,
  aceleración y términos P/I/D por eje.
- Tabla con cabecera y las 34 variables enviadas por el firmware.
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
4. Las 34 señales en el mismo orden de `TELEMETRY.md`.

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
- **Candidatos inválidos:** sync encontrada con versión distinta de 1 o valores
  flotantes no finitos.
- **Overflow cola PC:** la interfaz no consumió eventos tan rápido como llegaron.

La trama no incluye CRC. La aplicación comprueba sync, versión y valores
finitos, pero un byte alterado que todavía produzca un `float32` finito puede
pasar inadvertido. Para análisis de control, confirme también continuidad de
secuencia y timestamp.
