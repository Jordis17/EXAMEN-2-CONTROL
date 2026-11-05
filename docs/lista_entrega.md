# Lista de verificación para la entrega (13 de noviembre de 2026, Teams)

## Archivos a entregar
- [ ] `ValidacionIA_ConvertidorDCDC.mlx` (Live Script convertido desde `matlab/ValidacionIA_ConvertidorDCDC.m`)
- [ ] `ConvertidorDCDC_ControlObservador.slx` (generado con `simulink/construir_modelo_simulink.m`)
- [ ] `informe.pdf` (compilado desde `informe/informe.tex`)
- [ ] Opcional: `construir_modelo_simulink.m` y `graficas_simulink.m`

## Rúbrica
### Prompt e IA (15 %)
- [ ] Prompt final en el informe (sección VI) idéntico al usado
- [ ] Herramienta, plan gratuito, modelo y fecha documentados
- [ ] Respuesta íntegra en el Apéndice A
- [ ] Resumen de dificultades
- [ ] Prompt probado de nuevo en una conversación nueva (el profesor lo va a repetir)

### Live Script (25 %)
- [ ] Sección 2 llena con los valores de la IA
- [ ] Corre completo con "Run All" después de `clear all` en una sesión nueva
- [ ] Conversión a .mlx revisada (títulos, texto y ecuaciones visibles)
- [ ] Veredicto final coherente con la tabla de comparación

### Simulink (35 %)
- [ ] El .slx abre y simula sin correr ningún script antes
- [ ] CI de la planta [2; 1] y del observador [0; 0] (distintas)
- [ ] Diagrama ordenado y legible para la captura
- [ ] Scopes muestran convergencia y salida igual a 1 V

### Informe (15 %) y formato (10 %)
- [ ] Ninguna marca "PENDIENTE" en el .tex
- [ ] Valores de la Tabla III confirmados con la consola de `graficas_simulink.m`
- [ ] Figuras exportadas desde MATLAB (fondo blanco, ejes con unidades, leyendas)
- [ ] Nombres de los integrantes
- [ ] Revisión ortográfica final
