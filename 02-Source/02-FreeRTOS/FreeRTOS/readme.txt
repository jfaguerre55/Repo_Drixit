Version: FreeRTOS Kernel V10.0.1
Compiler: GCC
Architecture: ARM Cortex M4-F o M3


Esta distribución de FreeRTOS se puede compilar como biblioteca estática. El kernel está completamente contenido en /src y /inc.

Para poder compilar como una bilioteca estática FreeRTOS necesita el archivo FreeRTOSConfig.h que configura varias opciones del SO 
que se definen en tiempo de compilación. Este archivo depende de las necesidades del proyecto ya que configura cómo se va a utilizar el SO.

Además, la distribución incluye (en /src/portable/MemMang) cinco modelos de manejo del heap de FreeRTOS, deben borrarse 4 y dejar el que se 
vaya a usar en la aplicación. Las opciones de heap se ven en: https://www.freertos.org/a00111.html.

Por lo tanto, está distribución de FreeRTOS debe copiarse como un proyecto standalone y configurarse según las necesidades de la aplicación.
Si se usa directamente desde Lib_Firmware_Embebido se corre el riesgo que un proyecto pise la configuración del SO de otro.

Es decir, esta distribucion de FreeRTOS es una TEMPLATE !!!! Cada aplicación debe copiarse el template y cutomizarlo según sus necesidades.

 
