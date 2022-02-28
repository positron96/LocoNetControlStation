Import('env')
from SCons.Script import COMMAND_LINE_TARGETS  # pylint: disable=import-error

if '__test' in COMMAND_LINE_TARGETS:
    #print(env.get('SRC_FILTER'))
    env.Replace(SRC_FILTER=["+<*>", "-<DCC.cpp>"])

# pass flags to a global build environment (for all libraries, etc)
# global_env = DefaultEnvironment()
# global_env.Append(
#     CPPDEFINES=[
#         ("MQTT_MAX_PACKET_SIZE", 512),
#         "ARDUINOJSON_ENABLE_STD_STRING",
#         ("BUFFER_LENGTH", 32)
#     ]
# )