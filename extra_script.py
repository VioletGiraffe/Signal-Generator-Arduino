Import("env")

# C++ only
env.Append(CXXFLAGS=["-Wno-register"])
# C only
#env.Append(CCFLAGS=["-std=c11"])