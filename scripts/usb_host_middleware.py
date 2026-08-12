#
# Pulls the ST USB Host *Core* library straight out of the framework-stm32cubef4
# package and compiles it into the build, without copying files into the repo.
#
# We deliberately do NOT include the ST HID class driver (Class/HID): its
# USBH_HID_InterfaceInit() rejects any HID interface that is not a boot-protocol
# mouse/keyboard, which is exactly what a force-feedback wheel is.  The custom
# class in src/g29_hid.c replaces it.
#
import os

Import("env")  # noqa: F821

platform = env.PioPlatform()  # noqa: F821

# The FPU flags in platformio.ini reach the compiler but not the linker, and a
# hard-float object cannot be linked into a soft-float image ("uses VFP
# register arguments"). Mirror them into LINKFLAGS so the whole image agrees.
env.Append(LINKFLAGS=["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"])  # noqa: F821
FRAMEWORK_DIR = platform.get_package_dir("framework-stm32cubef4")
assert FRAMEWORK_DIR and os.path.isdir(FRAMEWORK_DIR), \
    "framework-stm32cubef4 not found — run a build once to download it"

usbh_core = os.path.join(
    FRAMEWORK_DIR, "Middlewares", "ST", "STM32_USB_Host_Library", "Core"
)
assert os.path.isdir(usbh_core), \
    "USB Host Library not present in framework: %s" % usbh_core

# Headers (usbh_core.h, usbh_def.h, usbh_ctlreq.h, usbh_ioreq.h, usbh_pipes.h)
env.Append(CPPPATH=[os.path.join(usbh_core, "Inc")])  # noqa: F821

# Compile usbh_core.c, usbh_ctlreq.c, usbh_ioreq.c, usbh_pipes.c.
# Exclude usbh_conf_template.c — it defines USBH_LL_* which we provide in
# src/usbh_conf.c (would otherwise be duplicate symbols).
env.BuildSources(  # noqa: F821
    os.path.join("$BUILD_DIR", "usbh_core"),
    usbh_core,
    src_filter=["+<Src/*.c>", "-<Src/usbh_conf_template.c>"],
)

print("[usb_host_middleware] USB Host Core from: %s" % usbh_core)

#
# FreeRTOS, pulled the same way. Native API only — the CMSIS_RTOS wrapper is
# skipped because nothing needs it: USBH_USE_OS stays 0 (the G29 class polls
# URB state, so the core must be serviced continuously rather than woken by a
# message queue) and the application calls FreeRTOS directly.
#
freertos = os.path.join(FRAMEWORK_DIR, "Middlewares", "Third_Party", "FreeRTOS", "Source")
assert os.path.isdir(freertos), "FreeRTOS not present in framework: %s" % freertos

port_dir = os.path.join(freertos, "portable", "GCC", "ARM_CM4F")  # F401 = CM4F

env.Append(CPPPATH=[  # noqa: F821
    os.path.join(freertos, "include"),
    port_dir,
])

# tasks/queue/list are mandatory; timers and event_groups are cheap and likely
# wanted as the project grows. croutine and stream_buffer are left out.
env.BuildSources(  # noqa: F821
    os.path.join("$BUILD_DIR", "freertos"),
    freertos,
    src_filter=[
        "+<tasks.c>", "+<queue.c>", "+<list.c>",
        "+<timers.c>", "+<event_groups.c>",
    ],
)
env.BuildSources(  # noqa: F821
    os.path.join("$BUILD_DIR", "freertos_port"), port_dir, src_filter=["+<*.c>"],
)
# heap_4: coalescing first-fit. Everything is allocated once at boot, so the
# allocator barely matters, but heap_4 is the one that tolerates freeing.
env.BuildSources(  # noqa: F821
    os.path.join("$BUILD_DIR", "freertos_heap"),
    os.path.join(freertos, "portable", "MemMang"),
    src_filter=["+<heap_4.c>"],
)

print("[usb_host_middleware] FreeRTOS (ARM_CM4F, heap_4) from: %s" % freertos)
