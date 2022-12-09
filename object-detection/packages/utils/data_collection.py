import cv2
import numpy as np

# ==> Important - don't remove these imports even though they seem unneeded
import pyglet
from pyglet.window import key

# <== Important - don't remove these imports even though they seem unneeded

from utils.constants import ASSETS_DIR
from utils.agent import PurePursuitPolicy
from utils.misc import launch_env, seed, xminyminxmaxymax2xywfnormalized, train_test_split

from solution.setup_activity import find_all_boxes_and_classes

# -----------------------------------------------------------------------------
# NOTE: These are parameters you can play with

# - the percentage of simulated data that will go into the training set (as opposed to the testing set)
SIMULATED_TRAIN_SPLIT_PERCENTAGE = 0.8

# - maps to use while generating new images (you don't have to change this)
MAPS = ["loop_pedestrians", "udem1", "loop_dyn_duckiebots", "zigzag_dists"]

# - max number of images total and per map
MAX_NUMBER_OF_IMAGES = 200
MAX_NUMBER_OF_IMAGES_PER_MAP = 50

# - every X images taken from a map we reset the environment
RESET_ENVIRONMENT_EVERY_IMAGES = 100

# - save images with bounding boxes (this will NOT work for training, but is useful for debugging)
EXPORT_BOUNDING_BOX_IMAGES = False

# - opens the simulator window
SHOW_SIMULATOR_WINDOW = True

# NOTE: Do not change below this line
# -----------------------------------------------------------------------------

# setup environment
seed(123)
# constants
DATASET_DIR = f"{ASSETS_DIR}/duckietown_object_detection_dataset"
IMAGE_SIZE = 416


class SkipException(Exception):
    pass


def add_image(_img, _boxes, _classes, _index, _progress_str):
    _img = cv2.cvtColor(_img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"{DATASET_DIR}/images/{_index}.jpg", _img)
    _boxes = np.array([xminyminxmaxymax2xywfnormalized(_box, IMAGE_SIZE) for _box in _boxes])
    with open(f"{DATASET_DIR}/labels/{_index}.txt", "w") as f:
        for i in range(len(_boxes)):
            f.write(f"{_classes[i]} " + " ".join(map(str, _boxes[i])) + "\n")
    # e.g. _progress_str = "[Map <map_name> | (i/N) ] - (curr map) #i / N - (total) #i / N"
    print(f"Progress: {_progress_str}", end="\r", flush=True)


def _num_images_per_map():
    """Return a list indicating how many images per map will be generated"""

    # e.g. 35 total, 10 max per map, 4 maps: 3 maps with 10 images and 1 map with 5
    # [10, 10, 10, 5]

    # e.g. 10 total, 20 max per map, 4 maps: 1 map with 10 images
    # [10, 0, 0, 0]

    # e.g. 80 total, 10 max per map, 4 maps: 4 map with 10 images
    # [10, 10, 10, 10]

    _quotient = MAX_NUMBER_OF_IMAGES // MAX_NUMBER_OF_IMAGES_PER_MAP
    _remainder = MAX_NUMBER_OF_IMAGES % MAX_NUMBER_OF_IMAGES_PER_MAP
    _n_maps = len(MAPS)

    if _quotient >= _n_maps:  # all maps will have max num images
        return [MAX_NUMBER_OF_IMAGES_PER_MAP] * _n_maps

    ret = [MAX_NUMBER_OF_IMAGES_PER_MAP] * _quotient + [_remainder]
    _len_tmp = len(ret)

    for _itr in range(_n_maps - _len_tmp):  # no need to use remaining maps
        ret.append(0)

    return ret


if __name__ == "__main__":
    # number of images collected in total
    no_images_so_far = 0

    lst_num_images_per_map = _num_images_per_map()
    total_target_num_images = sum(lst_num_images_per_map)

    # an env is a map. We iterate over maps to get diverse data
    env = None
    for env_id, target_num_images_curr_map in enumerate(lst_num_images_per_map):
        # set up new map
        if env is not None:
            try:
                env.window.close()
                env.close()
            except:
                pass

        if env_id >= len(MAPS):  # should never happen
            env_id = env_id % len(MAPS)
        env = launch_env(MAPS[env_id])
        policy = PurePursuitPolicy(env)
        obs = env.reset()

        # generate data with this env
        env_step = 0
        while no_images_so_far < MAX_NUMBER_OF_IMAGES and env_step < target_num_images_curr_map:
            action = policy.predict(np.array(obs))

            obs, rew, done, misc = env.step(action)
            seg = env.render_obs(True)

            obs = cv2.resize(obs, (IMAGE_SIZE, IMAGE_SIZE))
            seg = cv2.resize(seg, (IMAGE_SIZE, IMAGE_SIZE))

            # render
            if SHOW_SIMULATOR_WINDOW:
                env.render(segment=True)

            try:
                boxes, classes = find_all_boxes_and_classes(seg)
            except SkipException as e:
                print(e)
                continue

            if EXPORT_BOUNDING_BOX_IMAGES:
                for box in boxes:
                    pt1 = (box[0], box[1])
                    pt2 = (box[2], box[3])
                    cv2.rectangle(obs, pt1, pt2, (255, 0, 0), 2)

            _st_map = f"map'{MAPS[env_id]}'|{env_id + 1}/{len(MAPS)}"
            _st_curr = f"(curr)#{env_step + 1}/{target_num_images_curr_map}"
            _st_total = f"(total)#{no_images_so_far + 1}/{total_target_num_images}"

            progress_str = f"[{_st_map}] {_st_curr} - {_st_total}"
            add_image(obs, boxes, classes, no_images_so_far, progress_str)
            no_images_so_far += 1
            env_step += 1

            if done or env_step % RESET_ENVIRONMENT_EVERY_IMAGES == 0:
                env.reset()

        if no_images_so_far >= MAX_NUMBER_OF_IMAGES:
            break

    print("NOW GOING TO MOVE IMAGES INTO TRAIN AND VAL")
    all_image_names = [str(idx) for idx in range(no_images_so_far)]
    train_test_split(all_image_names, SIMULATED_TRAIN_SPLIT_PERCENTAGE, DATASET_DIR)
    print("DONE!")
