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

# setup environment
seed(123)
# constants
DATASET_DIR = f"{ASSETS_DIR}/duckietown_object_detection_dataset"
IMAGE_SIZE = 416

# NOTE: These are parameters you can play with
# - the percentage of simulated data that will go into the training set (as opposed to the testing set)
SIMULATED_TRAIN_SPLIT_PERCENTAGE = 0.8
# - maps to use while generating new images
MAPS = ["loop_pedestrians", "udem1", "loop_dyn_duckiebots", "zigzag_dists"]
# - max number of images total and per map
MAX_NUMBER_OF_IMAGES = 1000
MAX_NUMBER_OF_IMAGES_PER_MAPS = 100
# - every X images taken from a map we reset the environment
RESET_ENVIRONMENT_EVERY_IMAGES = 100
# - save images with bounding boxes (this will NOT work for training, but is useful for debugging)
EXPORT_BOUNDING_BOX_IMAGES = False
# - opens the simulator window
SHOW_SIMULATOR_WINDOW = True


class SkipException(Exception):
    pass


def add_image(_img, _boxes, _classes, _index):
    _img = cv2.cvtColor(_img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"{DATASET_DIR}/images/{_index}.jpg", _img)
    _boxes = np.array([xminyminxmaxymax2xywfnormalized(_box, IMAGE_SIZE) for _box in _boxes])
    with open(f"{DATASET_DIR}/labels/{_index}.txt", "w") as f:
        for i in range(len(_boxes)):
            f.write(f"{_classes[i]} " + " ".join(map(str, _boxes[i])) + "\n")
    print(f"COLLECTED IMAGE #{_index}")


if __name__ == '__main__':
    # we interate over several maps to get more diverse data
    env_id = 0
    env = None
    no_images_so_far = 0

    while True:
        if env is not None:
            try:
                env.window.close()
                env.close()
            except:
                pass

        if env_id >= len(MAPS):
            env_id = env_id % len(MAPS)
        env = launch_env(MAPS[env_id])
        policy = PurePursuitPolicy(env)
        obs = env.reset()

        env_step = 0
        if no_images_so_far >= MAX_NUMBER_OF_IMAGES:
            break

        while True:
            if no_images_so_far >= MAX_NUMBER_OF_IMAGES or env_step > MAX_NUMBER_OF_IMAGES_PER_MAPS:
                break

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

            add_image(obs, boxes, classes, no_images_so_far)
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
