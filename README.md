# Morpheme Runtime 5.2 with Jolt Physics & euphoria

# current issues:

- ragdoll with euphoria seems too stiff, ex: frequent hopping when recovering balance. need to do proper tweaking to joint strength and damping, also figure out a way to implement internal & external compliance if needed so that, for example, the physics rig can strongly swing the leg, but weakly lift the body off the ground;

- constrained bodies do not work yet;

- some memory leaks, environment doesnt restart properly