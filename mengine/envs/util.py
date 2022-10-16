import os, sys
import numpy as np
import pybullet as p

class Util:
    def __init__(self, pid, np_random):
        self.id = pid
        self.np_random = np_random

    def enable_gpu(self):
        import GPUtil as GPU
        os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
        os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '330'
        enableGPU = False
        # Get all device ids and their processing and memory utiliazion
        # (deviceIds, gpuUtil, memUtil) = GPU.getGPUs()
        # Print os and python version information
        print('OS: ' + sys.platform)
        print(sys.version)
        # Print package name and version number
        print(GPU.__name__ + ' ' + GPU.__version__)

        # Show the utilization of all GPUs in a nice table
        GPU.showUtilization()

        # Show all stats of all GPUs in a nice table
        GPU.showUtilization(all=True)

        # NOTE: If all your GPUs currently have a memory consumption larger than 1%, this step will fail. It's not a bug! It is intended to do so, if it does not find an available GPU.
        GPUs = GPU.getGPUs()
        numGPUs = len(GPU.getGPUs())
        print("numGPUs=",numGPUs)
        if numGPUs > 0:
            enableGPU = True
        eglPluginId = -1
        if enableGPU:
            import pkgutil
            egl = pkgutil.get_loader('eglRenderer')
            if (egl):
                eglPluginId = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin", physicsClientId=self.id)
            else:
                eglPluginId = p.loadPlugin("eglRendererPlugin", physicsClientId=self.id)

        if eglPluginId>=0:
            print("Using GPU hardware (eglRenderer)")
        else:
            print("Using CPU renderer (TinyRenderer)")

