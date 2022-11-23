# Write a path planner

In this exercise, you will write a path planner. You will need to have solved the `collision` part already because you will use your collision checker (you probably want to copy it into your solution folder).

Note: This is a code-only exercise: you don't need the Duckiebot.


## Instructions

The template contained in the `planner` subfolder is a fully functional (but wrong) solution.
You can try to evaluate/submit it right away.

Make sure you have an updated system using

```shell
dts desktop update
```

To evaluate the submission use (change the corresponing challenge if needed):

```shell
dts challenges evaluate --challenge mooc-planning-dd-static-vali
```

To submit, use

```shell
dts challenges submit
```

This will send it to all the challenges listed in the `submission.yaml` file.

To minimize confusion, you might want to submit to one challenge at a time with the `--challenge` option.


### Passing criteria


To pass, you have to get at least 95% of the queries correct on the `*-test` challenges.
(This allows some slack, so that you can experiment with probabilistic algorithms).
