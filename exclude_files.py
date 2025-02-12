Import("env")

def exclude_file(env, node):
    print(f'Excluding {node} from build')
    # to ignore file from a build process, just return None
    return None

env.AddBuildMiddleware(exclude_file, "*/pro-rf-duplex.cpp")