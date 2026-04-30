Import("env")
import os

src = env.GetProjectOption("custom_src_dir")
env["PROJECT_SRC_DIR"] = os.path.join(env["PROJECT_DIR"], src)
