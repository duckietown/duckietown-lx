import duckietown_code_utils as dcu

from solution.image_filter import highlight_centerline

def test_filter_response():
    image_file_path = './assets/images/onboard.png'
    test_image = dcu.rgb_from_jpg_fn(image_file_path)

    filtered_image = highlight_centerline(test_image)
    assert filtered_image is not None
