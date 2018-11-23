****************
EXOTica overview
****************

The Extensible Optimization Toolset (EXOTica) is a framework of software tools designed for development and evaluation of motion synthesis algorithms within ROS. We will describe how to rapidly prototype new motion solvers that exploit a common problem definition and structure which facilitates benchmarking through modularity and encapsulation.
We will refer to several core concepts in robotics and motion planning throughout this chapter. These topics are well presented in robotics textbooks such as `Springer Handbook of Robotics <https://books.google.co.uk/books?id=Xpgi5gSuBxsC>`_ and `Planning Algorithms <http://planning.cs.uiuc.edu/>`_. This background material will help you to understand the area of research that motivated development of EXOTica. 

Our motivation to begin this work stems from the need to either implement new tools or to rely on existing software often designed for solving a problem other than the one we intended to study. The need to implement and test new ideas rapidly led us to the specification of a library that is modular and generic while providing useful tools for motion  planning. A guiding principle hereby is to remove implementation-specific bias when prototyping and comparing algorithms, and hitherto create a library of solvers and problem formulations. 

.. image:: images/example.png
    :width: 30%
    :align: right

In this chapter, we will use a well-known algorithm as an example in order to explain how to use and extend the core components of EXOTica to explore novel formulations. 
Consider a robot arm mounted to a workbench (see picture). The arm consists of several revolute joints actuated by servo motors moving the links of the robot body. A gripper may be attached to the final link. The task is to compute a single configuration of the robot arm which will place the gripper at a desired grasping position - i.e. our example will follow the implementation of an inverse kinematics solver. Once the problem and motion solver have been implemented, we can compute the robot configuration using EXOTica with the following code:

.. code-block:: cpp

    #include <exotica/Exotica.h>
    using namespace exotica;

    int main(int argc, char **argv)
    {
        MotionSolver_ptr solver = XMLLoader::loadSolver("{exotica_examples}/resources/configs/example.xml");
        Eigen::MatrixXd solution;
        solver->Solve(solution);
    }

This snippet shows how little code is required to run a motion planning experiment. We load a motion solver and a problem definition from an example configuration file located in the ``exotica_examples`` package, allocate the output variable, and solve the problem using three lines of code. What this snippet does not show is the definition of the planning problem, the implementation of the algorithm and an array of other tools available in EXOTica. The running example will focus on motion planning. However, we view motion planning and control as two approaches to solving the same motion synthesis problem at different scales. For example, the problem could be viewed as an end-pose motion planning problem as well as operational space control, when executed in a loop. This allows us to formulate complex control problems as re-planning and vice versa. EXOTica provides the tools to implement such systems.

To motivate and explain the EXOTica software framework, we focus on how it can be used in research and prototyping. We will do this by describing how problems and solvers are defined, and the various tools they use.

System overview
===============

.. image:: images/overview.png
    :width: 100%
    :align: center

Prototyping of novel motion planning algorithms relies on defining mathematical models of the robotic system and its environment. To aid this process, EXOTica provides several abstractions and generic interfaces that are used as components for building algorithms. The diagram above shows the three components central to algorithm design in EXOTica: (1) a ``planning scene``, providing tools to describe the state of the robot and the environment, (2) a ``planning problem`` formally defining the task, and (3) a ``motion solver``. These abstractions allow us to separate problem definitions from solvers. In particular, motion solvers implement algorithms such as `AICO <http://doi.acm.org/10.1145/1553374.1553508>`_ and `RRTConnect <https://ieeexplore.ieee.org/document/844730>`_. These implementations may perform trajectory optimization, randomized sampling, or any other computation which requires a very specific problem formulation.

How the problem is formulated is fully contained within the definition of a ``planning problem``. Each algorithm solves exactly one type of motion planning problem while one type of problem may be compatible with multiple solvers. As a result, several algorithms can be benchmarked on the exact same problem. When benchmarking two algorithms that are compatible with different types of problems, the problems have to be converted explicitly. This is a useful feature that makes it easy to track differences between problem formulations that are intended to describe the same task. 

All planning problems use the ``task maps`` as components to build cost functions, constraints, or validity checking criteria. Task maps perform useful computations such as forward kinematics, center-of-mass position calculation, and joint limit violation error computation. To further support the extensibility of EXOTica, the motion solvers and the task maps are loaded into EXOTica as plug-ins. As such, they can be developed separately and loaded on demand. One such example is the plug-in which wraps the sampling-based algorithms implemented in the `OMPL library <http://ompl.kavrakilab.org>`_.

The diagram above also shows the `planning scene` which separates the computation of kinematics from the computation of task related quantities.

System model
============
To synthesize motion, we describe the system consisting of the robot and its environment using a mathematical model. This system model may be kinematic or it may include dynamic properties and constraints. EXOTica uses the system model to evaluate the state using tools implemented inside the ``planning scene``. The system diagram shows the ``planning scene`` as a part of the planning problem where it performs several computations required for evaluating the problem.

Planning scene
--------------
The ``planning scene`` implements the tools for updating and managing the robot model and the environment. The robot model is represented by a kinematic tree which stores both the kinematic and dynamic properties of the robot, e.g., link masses and shapes, joint definitions, etc. The environment is a collection of additional models that are not part of the robot tree but that may interact with the robot. The environment may contain reference frames, other simplified models (geometric shapes), and real sensor data based representations such as pointclouds and `OctoMaps <http://octomap.github.com>`_. The planning scene implements algorithms for managing the objects in the environment (e.g. adding/removing obstacles) as well as computing forward kinematics and forward dynamics.

The system is parametrized by a set of variables that correspond to controllable elements, e.g. the robot joints. The full state of the system is described using these variables and we will refer to it as the ``robot state``. In some cases, only a subset of the robot state is controlled. We call this subset the ``joint group``. Analogous to the `MoveIt! <https://moveit.ros.org/>`_ definition of a move group, a joint group is a selection of controlled variables used for planning or control. From now on, whenever we refer to a joint state, we are referring to the state of the joint group.

The system model may be kinematic, kino-dynamic, or fully dynamic. The robot state is then described by joint positions, joint positions and velocities, or full system dynamics respectively. The system dynamics may be provided via a physics simulator. We will only consider the kinematic model for simplicity.

The system model is implemented as a tree structure mimicking the structure implemented in the `KDL library <http://www.orocos.org/kdl>`_. The diagram below illustrates the kinematic tree of a planar robot arm. 

.. image:: images/kinematic_tree.png
    :width: 100%
    :align: center

The planning scene stores the kinematic tree composed of the robot model and the environment. The diagram shows a robot model which has two revolute joints :math:`J_1` and :math:`J_2` defined by joint angles :math:`\theta_1` and :math:`\theta_2` respectively, a base frame and an end effector frame :math:`A`. A grasping target is located at frame :math:`B`. The root of the tree is at the world frame. The grasping task can exploit the relative transformation :math:`M_A^B`.

Every node in the tree has one parent and possibly multiple children. The node defines a spatial transformation from the tip frame of the parent node to its own tip frame. Every node consists of a position offset of the joint, a joint transformation, and a tip frame transformation (see the `KDL documentation <http://www.orocos.org/kdl>`_). The joint transformation is constant for fixed joints. The transformations of all joints that belong to the controlled ``joint group`` are updated based on the joint state. During the update, the local transformation of the node is updated and the transformation of the tip w.r.t. the world frame is accumulated. The nodes of the tree are updated in a topological order (from the root to the leafs). This ensures that the tip frame of the parent node is always updated before its children.

The EXOTica ``Scene`` implements a method for publishing the frames to `RViz <http://wiki.ros.org/rviz>`_ using `tf <http://wiki.ros.org/tf2>`_ for debugging purposes. These frames can be visualized using the `tf <http://wiki.ros.org/tf2>`_ and the ``RobotModel`` plug-ins.

The system model provides an interface to answer kinematic queries. A query can be submitted to the ``Scene``, requesting arbitrary frame transformations. Each requested frame has the following format:

 - Name of the tip frame (Frame A)
 - Offset of the tip frame
 - Name of the base frame (Frame B)
 - Offset the of base frame

The diagram above illustrates an example scene. Any existing frame can be used to define a base or a tip frame of a relative transformation.
The response to the query will then contain a transformation of the tip frame with respect to the base frame. If an offset is specified, each respective frame will be redefined to include the offset. If a base frame is not specified, the world frame will be used by default. Since all transformations of the tree nodes w.r.t. the world frame have been computed during the update, the query computation only adds the tip frame to the inverted base frame :math:`$M_A^B={M_B^{world}}^{-1}M_A^{world}`. 
We use the following notation: the subscript and superscript denote tip and base frames respectively. :math:`M_A^B` reads: transformation of frame :math:`A` w.r.t. frame :math:`B`.}


The ``Scene`` has been designed to answer a large number of requests in batches. While some smaller problems, such as simple kinematic chains, may be more costly to update, larger kinematic trees with a large number of leaf nodes are handled more efficiently by simply iterating over the requested frames.MoveIt!

The system model also computes derivatives of the spatial frames w.r.t. the control variables. These are computed as geometric Jacobians (:math:`J`) and Jacobian derivatives (:math:`\dot{J}`). The Jacobian has six rows and a number of columns corresponding to the number of controlled joints. Each column represents a spatial velocity in form of a ``twist``. The twist :math:`^Bt^i_A` describes the linear and angular rate of motion of the tip frame :math:`A` w.r.t. the joint frame :math:`i` expressed in the base frame :math:`B`. We use the notation with the ``expressed in frame`` in the left superscript. Using the twist representation allows us to correctly compute spatial transformations using the `Lie group algebra <http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf>`_.

The kinematic tree represents the robot kinematic model and the objects in the environment. The robot model can be loaded from a pair of `MoveIt! <https://moveit.ros.org/>`_ compatible URDF and SRDF files. The URDF file specifies the robot kinematics, joint transformations and range of motion, frame locations, mass properties and collision shapes. The SRDF file specifies the base of the robot (fixed, mobile, or floating), joint groups, and collision pairs. The robot configuration created for `MoveIt! <https://moveit.ros.org/>`_ is fully compatible with EXOTica. The ``Scene`` also implements an interface to populate the environment with collision objects from `MoveIt! <https://moveit.ros.org/>`_ planning scene messages and from `MoveIt! <https://moveit.ros.org/>`_ generated text files storing the scene objects. The ``Scene`` may load additional basic shape primitives, meshes, or `OctoMaps <http://octomap.github.com>`_. 

In order to perform collision checking, a ``CollisionScene`` can be loaded as a plug-in into a ``Scene``. This allows for different implementations of collision checking algorithms to be used as required and does not tie EXOTica to a particular collision checking library. For instance, by default, EXOTica ships with two ``CollisionScene`` implementations using the FCL library - one based on the stable FCL version also used in `MoveIt! <https://moveit.ros.org/>`_ and one tracking the development revision of FCL. The ``CollisionScene`` plug-ins may hereby implement solely binary collision checking, or additional contact information such as signed distance, contact (or nearest) points, as well as contact point normals. This information is captured and exposed in a so-called ``CollisionProxy``.

Referring back to the example inverse kinematics problem, the planning scene consists of the kinematics of the robot with a base link rigidly attached to the world frame. We choose to use a simplified version following the DH parameters of the KUKA LWR3 arm which we load from a pair of URDF and SRDF files. This robot has seven revolute joints. The joint group will consist of all seven joints as we intend to control all of them. We will not be performing collision checking in this experiment. The ``planning scene`` is initialized from an EXOTica XML configuration file. The XML file contains the following lines related to the setup of the ``planning scene``:

.. code-block:: xml

    <PlanningScene>
        <Scene>
            <JointGroup>arm</JointGroup>
            <URDF>{exotica_examples}/resources/robots/lwr_simplified.urdf</URDF>
            <SRDF>{exotica_examples}/resources/robots/lwr_simplified.srdf</SRDF>
        </Scene>
    </PlanningScene>

where the joint group parameter selects a joint group defined in the SRDF file by name. The robot model is loaded from the URDF and SRDF files specified here. When the paths are not specified, EXOTica attempts to load the robot model from the ``robot_description`` ROS parameter by default. EXOTica additionally allows to set ROS parameters for the planning robot description from specified file paths if desired.

The system model provides access to some generic tools for computing kinematic and dynamic properties of the system. These tools have been designed for performing calculations for solving a wide variety of motion planning problems. The system modeling tools are generic but they can be ultimately replaced with a more specific set of kinematics and dynamics solvers in the final deployment of the algorithm. This is, however, outside of the scope of EXOTica.

Problem definition
==================
EXOTica was designed for prototyping and benchmarking motion synthesis algorithms. The main objective of our framework is to provide tools for constructing problems and prototyping solvers with ease. To do so, we first separate the definition of the problem from the implementation of the solver. Each problem consists of several standardized components which we refer to as ``task maps``.

Task maps
---------
The core element of every problem defined within EXOTica is the function mapping from the configuration space (i.e. the problem state which captures the model state, a set of controlled and uncontrolled variables, and the state of the environment) to a task space. We call this function a ``task map``. For example, a task map computes the center-of-mass of the robot in the world frame. A task map is a mapping from the configuration space to an arbitrary task space. The task space is, in fact, defined by the output of this function. Several commonly used task maps are implemented within EXOTica.

Joint position
    task map computes the difference between the current joint configuration and a reference joint configuration:

    .. math::

        \Phi_\text{Ref}(\boldsymbol{x}) = \boldsymbol{x}-\boldsymbol{x}_{\text{ref}},

    where :math:`\boldsymbol{x}` is state vector of the joint configuration and :math:`\boldsymbol{x}_{\text{ref}}` is the reference configuration. The whole state vector :math:`x` may be used or a subset of joints may be selected. This feature is useful for constraining only some of the joints, e.g. constraining the back joints of a humanoid robot while performing a manipulation task. The Jacobian and Jacobian derivative are identity matrices. 

        We use notation :math:`x` for scalar values, :math:`\boldsymbol{x}` for vectors, :math:`X` for matrices, and :math:`\boldsymbol{X}` for vectorized matrices.

Joint limits 
    task map assigns a cost for violating joint limits. The joint limits are loaded from the robot model. The mapping is calculated as:

    .. math::

        \Phi_\text{Bound}(x) = 
        \begin{cases}
        x - x_\text{min} - \epsilon, & \text{if } x < x_\text{min}+\epsilon \\
        x - x_\text{max} + \epsilon, & \text{if } x > x_\text{max}-\epsilon \\
        0,                       & \text{otherwise}
        \end{cases},

where :math:`x_\text{min}` and :math:`x_\text{max}` are lower and upper joint limits respectively, and :math:`\epsilon\geq0` is a safety margin. The Jacobian and Jacobian derivative are identity matrices. 

End-effector frame
    task map captures the relative transformation between the base frame :math:`B` and the tip frame :math:`A`:

    .. math::

        \Phi_\text{EffFrame}(\boldsymbol{x}) = \boldsymbol{M}_A^B,

    where :math:`\boldsymbol{M}_A^B\in SE(3)` is computed using the system model using the ``Scene``. We use the ``task space vector`` data structure (described later in this section) to handle storage and operations on spatial frames. The Jacobian of this task map is the geometric Jacobian computed by the ``Scene``.

End-effector position
    captures the translation of the relative frame transformation:

    .. math::

        \Phi_\text{EffPos}(\boldsymbol{x}) = \boldsymbol{P}_A^B,

    where :math:`\boldsymbol{P}_A^B` is translational part of :math:`\boldsymbol{M}_A^B`. The Jacobian of this task consists of the rows of the geometric Jacobian corresponding to the translation of the frame.

End-effector orientation
    captures the rotation of the relative frame transformation:

    .. math::

        \Phi_\text{EffRot}(\boldsymbol{x}) = \boldsymbol{R}_A^B,

where :math:`\boldsymbol{R}_A^B\in SO(3)` is rotational part of :math:`\boldsymbol{M}_A^B`. Similarly to the ``end-effector frame`` task map, the storage and the operations on the resulting :math:`SO(3)` space are implemented within the ``task space vector``. The Jacobian of this task consists of the rows of the geometric Jacobian corresponding to the rotation of the frame.

End-effector distance
    computes the Euclidean distance between the base and tip frames:

    .. math::

        \Phi_\text{Dist}(\boldsymbol{x}) = ||\boldsymbol{P}_A^B||.

    The resulting task map has the same function as the ``end-effector position`` map. The output, however, is a scalar distance.

Center-of-mass
    task map computes the center-of-mass of all of the robot links defined in the system model:

    .. math::

        \Phi_\text{CoM}(\boldsymbol{x}) = \sum_i(\boldsymbol{P}_{\text{CoM}_i}^\text{world}m_i),

    where :math:`\boldsymbol{P}_{\text{CoM}_i}^\text{world}` is the position of the center-of-mass of the :math:`i`-th link w.r.t. the world frame, and :math:`m_i` is mass of the :math:`i`-th body. The Jacobian is computed using the chain rule. This task map can also be initialized to compute the projection of the center-of-mass on the :math:`xy`-plane. In this case, the :math:`z`-component is removed.

Collision spheres
    task map provides a differentiable collision distance metric. The collision shapes are approximated by spheres. Each sphere is attached to the kinematic structure of the robot or to the environment. Each sphere is then assigned a collision group, e.g. :math:`i\in \mathcal{G}`. Spheres within the same group do not collide with each other, while spheres from different groups do. The collision cost is computed as:

    .. math::

        \Phi_\text{CSphere}(\boldsymbol{x}) = \sum_{i,j}^{G}\frac{1}{1+ e^{5\epsilon (||\boldsymbol{P}_i^\text{world}-\boldsymbol{P}_j^\text{world}||-r_i-r_j)} }, 

    where :math:`i, j` are indices of spheres from different collision groups, :math:`\epsilon` is a precision parameter, :math:`\boldsymbol{P}_i^\text{world}` and :math:`\boldsymbol{P}_j^\text{world}` are positions of the centers of the spheres, and :math:`r_i, r_j` are the radii of the spheres. The sigmoid function raises from :math:`0` to :math:`1`, with the steepest slope at the point where the two spheres collide. Far objects contribute small amount of error while colliding objects produce relatively large amounts of error. The precision parameter can be used to adjust the fall-off of the error function, e.g. a precision factor of :math:`10^3` will result in negligible error when the spheres are further than :math:`10^{-3}`m apart. The constant multiplier of :math:`5` in was chosen to achieve this fall-off profile.

In our example, we use the ``end-effector position`` task map. The task space is therefore :math:`\Phi_\text{EffPos}(\boldsymbol{x})\in\mathbb{R}^3`. The task map is loaded from the XML file. The following lines of the XML configuration file correspond to the task map definition:

.. code-block:: xml

    <Maps>
        <EffPosition Name="Position">
            <EndEffector>
            <Frame Link="lwr_arm_7_link" BaseOffset="0.5 0 0.5 0 0 0 1"/>
            </EndEffector>
        </EffPosition>
    </Maps>

where the only parameter of the task map is a single relative spatial frame. This frame defines the translation of the seventh robot link relative to the coordinates :math:`(0.5, 0, 0.5)` in the world frame. If no frame is specified, world frame is assumed by default. If a relative offset is not specified, an identity transformation offset is assumed. This example is only intended to compute inverse kinematics, we have therefore chosen to only minimize the end-effector position error. However, an arbitrary number of cost terms can be added by adding multiple task maps to this problem definition. For instance, we could easily add another task map to constrain the orientation of the end-effector.

The output of a task map is a representation of the robot's state in the task space. Most task spaces are :math:`\mathbb{R}^n`. As such, they can be stored and handled as vectors of real numbers. However, some task maps output configurations in the :math:`SO(3)` or the :math:`SE(3)` space. In this case, `Lie group algebra <http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf>`_ has to be used to correctly compute the additions and subtractions in the task space. The ``task space vector`` implements operations on task spaces. The ``task space vector`` is a data structure that keeps track of :math:`SO(3)` sub-groups within the stored vector. The operations on this vector then implement the Lie group algebra. For example, a spatial frame may be stored as a transformation matrix :math:`M_A^B\in\mathbb{R}^{4\times4}`. This matrix will be stored in the ``task space vector``. Performing addition and subtraction on the vector will then be incorrect. The correct transformation is performed by a matrix multiplication. The ``task space vector`` keeps track of transformations stored within its structure and applies the correct operations on them. Furthermore, the result of subtraction is always a geometric twist, e.g., :math:`M_A^B-M_C^B={^B\boldsymbol{t}_A^C}`. This makes it possible to multiply the result of this operation with a geometric Jacobian, producing a geometrically correct relative transformation. This feature has been used in the implementation of the `inverse kinematics solver <http://dynamicsystems.asmedigitalcollection.asme.org/article.aspx?articleid=1403812>`_ and the `AICO solver <http://doi.acm.org/10.1145/1553374.1553508>`_. Additionally,
a :math:`SO(3)` rotation can be represented and stored in different ways, e.g. as a unit quaternion :math:`\boldsymbol{R}_\mathcal{Q}\in\mathbb{R}^4 \text{ where } ||\boldsymbol{R}_\mathcal{Q}||=1`, Euler angles :math:`\boldsymbol{R}_\mathcal{ZYZ},\boldsymbol{R}_\mathcal{ZYX},\boldsymbol{R}_\mathcal{RPY}\in\mathbb{R}^3`, angle-axis representation :math:`\boldsymbol{R}_\mathcal{A}\in\mathbb{R}^3 \text{ where } ||R_\mathcal{A}||=\theta`, rotation matrix :math:`\boldsymbol{R}\in\mathbb{R}^{3\times3}`, etc. We handle these representations implicitly. Each sub-group of the ``task space vector`` stores the size and type of representation that was used. The operations on the vector first convert the task space coordinates into a rotation matrix representation, then the correct spatial operation is applied and a twist is computed. As a result the input and output dimension may vary, i.e. subtraction of two rotations represented as rotation matrices is a function :math:`f(R_1, R_2): \mathbb{R}^9 \rightarrow \mathbb{R}^3`. The result is the angular velocity component of the twist. The ``task space vector`` is composed by concatenating outputs of multiple task maps. Each task map specifies if its output contains any components that have to be handled using the Lie group algebra.

.. image:: images/taskspace.png
    :width: 100%
    :align: center

Task space vector data packing combining three position coordinates :math:`x, y, z\in \mathbb{R}` and a sub-vector containing a :math:`SO(3)` rotation represented as a unit quaternion. The subtraction calculation of two task space vectors :math:`\boldsymbol{y}_1` and :math:`\boldsymbol{y}_2` first converts the quaternions into rotation matrices :math:`R_1` and :math:`R_2` and performs the rotation operation :math:`R_2^{-1}R_1`. The result is then converted into angular velocities :math:`\omega_x, \omega_y, \omega_z` and packed into the output vector :math:`\vartriangle\!\!y`. Notice that the dimensionality of :math:`\vartriangle\!\!\boldsymbol{y}\in\mathbb{R}^6` and :math:`\boldsymbol{y}_1, \boldsymbol{y}_2\in\mathbb{R}^7` are different.

The output of a single task map is a segment of the ``task space vector``. The input of a task map is the states of the robot model and environment as well as the arbitrary number of frame transformations required for the calculations. These are computed using the ``planning scene``. The task map implements the mapping within its ``update`` method. This method has 3 different overloads depending on what order of derivative is requested: a) no derivative (e.g. in sampling), b) first-order derivatives (e.g. Jacobian used in gradient descent), and c) second-order derivatives. Not all overloads have to be defined, i.e. a collision checking task map may only detect collisions but it will not provide any gradients (derivatives). We exploit this for fast collision checking for `sampling-based solvers <http://planning.cs.uiuc.edu/>`_.

The task map will update the task space vector and its derivatives when the solver requires it. These updates are normally triggered by the solver and they do not have to be called manually. This also ensures that the ``task space vector`` is updated correctly. The collection of task maps is therefore central to formally defining motion planning problems. How the output of the task map is used then depends on the type of the planning problem.

Planning problems
=================