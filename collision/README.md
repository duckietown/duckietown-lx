
# Write a collision checker

In this exercise, you will write a collision checker. In the next exercise you will use this collision checker as part of a planning algorithm.

Note: This is a code-only exercise: you don't need the Duckiebot.


* [Instructions](#instructions)
* [Data structures and protocol](#data-structures-and-protocol)
* [Template](#template)
* [Tips for implementing the collision checker](#tips-for-implementing-the-collision-checker)
  + [Use decomposition](#use-decomposition)
  + [Pay attention to the poses](#pay-attention-to-the-poses)
  + [In the end, what is the core complexity?](#in-the-end--what-is-the-core-complexity-)
  + [Speeding things up using lower/upper bound heuristics](#speeding-things-up-using-lower-upper-bound-heuristics)
  + [Speeding things up using bitmaps heuristics](#speeding-things-up-using-bitmaps-heuristics)


## Instructions

The template contained in this folder is a fully functional (wrong) solution. 
You can try to evaluate/submit right away.

Make sure you have an updated system using

    dts desktop update

To evaluate the submission,  go in `collision_checker` and use:

    dts challenges evaluate --challenge mooc-collision-check-vali 

To submit, use

    dts challenges submit

Note that the submission will be sent to two different challenges:

- [`mooc-collision-check-vali`][vali] is the **validation** challenge. You will be able to see the score and other output.
- [`mooc-collision-check-test`][test] is the **testing** challenge. You will not be able to see the scores.

To pass, you have to get at least 95% of the queries correct on the `mooc-collision-check-test` challenge. (This allows some slack, so that you can experiment with probabilistic algorithms).

Note that you cannot do

    dts challenges evaluate --challenge mooc-collision-check-test  !! does not work !!

because the test challenge must remain a secret.

[test]: https://challenges.duckietown.org/v4/humans/challenges/mooc-collision-check-test
[vali]: https://challenges.duckietown.org/v4/humans/challenges/mooc-collision-check-vali

Now, go read `collision_checker.py`, which contains the template. There are some hints to get started there. 

Modify the file and test if the program runs or check its performance with local evaluations over the validation dataset. And once satisfied with the program, submit to the challenges.

## Data structures and protocol

The data structures are defined in the `dt-protocols-daffy` package, which you can install via `pip`, or directly clone from [repo][repo].


[repo]: https://github.com/duckietown/dt-protocols


In particular, you can see in [`collision_protocol.py`][file] the data structures to use.

[file]: https://github.com/duckietown/dt-protocols/blob/daffy/src/dt_protocols/collision_protocol.py


The parameters for the collision checker is a `MapDefinition`, which specifies the `environment` and `body`.

Both `environment` and `body` are lists of `PlacedPrimitive`s.

A `PlacedPrimitive` is a pair of a `FriendlyPose` and a `Primitive`.

```python
@dataclass
class PlacedPrimitive:
    pose: FriendlyPose
    primitive: Primitive
    
    
@dataclass
class FriendlyPose:
    x: float
    y: float
    theta_deg: float
```


A `Primitive` is either a `Rectangle` or a `Circle`.

```python

@dataclass
class Circle:
    radius: float


@dataclass
class Rectangle:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

Primitive = Union[Circle, Rectangle]
```

So, we represents shapes as the union of rototranslated `Rectangle`s and `Circle`s.


The collision checker receives first a message `MapDefinition`, and then a sequence of `CollisionCheckQuery`s. The query contains a pose for the robot. The `CollisionCheckResult` contains only a boolean: true means that it is in collision, false means that it is not in collision.


## Template

In `collision_checker/collision_checker.py` you will find the template for the collision checker.

## Visualization

The challenges output will be a series of images.

In the `queries` folder you will see the queries with the ground truth,
as the image shows.

![query](sample/queries/env18.png)

Colors:

- Blue is a pose in which the robot does not collide.
- Red is a pose in which the robot collides.

In the `results` folder you will see your results and the errors you made:

![result](sample/results/env18-result.png)

The colors mean the following:

- Blue is a pose in which the robot does not collide and you guessed RIGHT.
- Orange is a pose in which the robot does not collide and you guessed WRONG.
- Red is a pose in which the robot collides and you guessed RIGHT.
- Pink is a pose in which the robot collides and you guessed WRONG.

## Tips for implementing the collision checker

There are multiple ways to implement the collision checker. Here are some tips, but feel free to follow your intuition.

### Use decomposition

The first thing to note is that the problem can be *decomposed*.

You are asked to see whether the robot collides with the environment at a certain pose.
Both robot and environment are lists of `Primitive`s. In pseudocode:

    robot =  rp1 ∪ rp2 ∪ ... 
    Wcoll =  wc1 ∪ wc2 ∪ ...

What you have to check is whether the intersection

    robot ∩ Wcoll 

is empty. Expanding:

    (rp1 ∪ rp2 ∪ ... ) ∩ (wc1 ∪ wc2 ∪ ...)

Now, the intersection of unions is a union of intersection:

    [rp1 ∩ (wc1 ∪ wc2 ∪ ...)]  ∪  [rp2 ∩ (wc1 ∪ wc2 ∪ ...)] ∪ ...

The above shows that you have to check whether any primitive of the robot collides with environment.

Further expanding the first term we obtain:

    [rp1 ∩ (wc1 ∪ wc2 ∪ ...)] = (rp1 ∩ wc1) ∪ (rp2 ∩ wc1) ∪ ...

which shows that in the end, you can reduce the problem to checking pairwise intersection of primitives.

### Pay attention to the poses

Both robot and environment are lists of **rototranslated** primitives.

That is, we should rewrite the robot expression as:

    robot = RT(pose1, primitive1) ∪ RT(pose2, primitive1) ∪ ...

where `RT()` rototranslates a primitive by a pose.

Also note that for each query the robot changes pose. Let's call this pose `Q`.

Note that we have

    robot at pose Q = RT(Q * pose1, primitive1) ∪ RT(Q * pose2, primitive1) ∪ ... 

where `Q * pose` represent matrix multiplication.

The above says that you can "push inside" the global pose.


### In the end, what is the core complexity?

Following the above tips, you should be able to get to the point where you are left with checking the collision of two rototranslated primitives.

Note that without loss of generality you can get to the point where you have one primitive at the origin. (You put one primitive in the coordinate frame of the other.)

Now notice that there are 3 cases:

- `Rectangle` vs `Circle`
- `Rectangle` vs `Rectangle`
- `Circle` vs `Circle`

`Circle` vs `Circle` is easy: two circles intersects if the distance of the centers is less than the sum of the radii.

For the others, you have to think about it...

 

### Speeding things up using lower/upper bound heuristics

If you want to speed things up, consider the following method, which allows to introduce a fast heuristic phase using only circle-to-circle comparisons.

For each rectangle `R`, you can find `C1`, the largest circle that is contained in the rectangle, and `C2`, the smallest circle that contains the rectangle. These are an upper bound and a lower bound to the shape.

    C1 ⊆ R ⊆ C2

Now notice that:

- if `C1` collides with a shape, also `R` does.  (but if it doesn't you cannot conclude anything)
- if `C2` does not collide with a shape, `R` does not as well. (but if it does, you cannot conclude anything)

Using this logic, you can implement a method that first checks quickly whether the circle approximations give already enough information to conclude collision/no-collision. Only if the first test is inconclusive you go to the more expensive component.

### Speeding things up using bitmaps heuristics

Another approach is using bitmaps to convert the environment to an image, where a black pixel means "occupied", and a white pixel means "free". 

Then you can do the same with the robot shape and obtain another bitmap.

Then you check whether the two bitmaps intersect

Advantages:

- reduces the problem of collision to drawing of shapes;
- cheaper if shapes are very complex.

Disadvantages:

- There are subtle issues regarding the approximations you are making. What exactly does a pixel represent? is it a point, or is it an area? is this an optimistic or pessimistic approximation? The semantics of painting is unclear. 


