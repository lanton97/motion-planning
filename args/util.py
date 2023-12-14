import os, datetime, imageio

# This function creates a new directory for storing results and returns the name
def generate_results_dir(map_name, alg_type, suffix='', timestamp_dir=True):
    dir = './results/' + map_name + '/' + alg_type + '/'
    dir = dir + suffix + '/' if suffix else dir
    dir = os.path.join(dir, datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S/')) if timestamp_dir else dir
    os.makedirs(dir, exist_ok=True)
    return dir

# Save an array of NP images as a gif
def save_gif(imgs, path='./example.gif'):
    with imageio.get_writer(path, mode='I',fps=30) as writer:
        for img in imgs:
            writer.append_data(img)
