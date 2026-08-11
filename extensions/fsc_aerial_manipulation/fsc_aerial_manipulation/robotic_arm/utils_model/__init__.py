"""
utils_model — one-shot USD asset-authoring scripts. NOT runtime components.

    postprocessor.py       OnShape -> USD cleanup for the gripper_bat arm+drone
                           (reparent / rename / realign frames / set mass and
                           inertia / add colliders / apply joint drives / export)
    x650_postprocessor.py  the same pipeline for the bare X650 frame; unlike the
                           arm postprocessor it authors no joint drives

Neither is imported by anything. They are pasted into Isaac Sim's Script Editor
after a CAD import and run once; the .usd/.usda they produce is what the
simulation actually loads. They live here so that "code that shapes the model"
is separable from code that flies it.
"""
