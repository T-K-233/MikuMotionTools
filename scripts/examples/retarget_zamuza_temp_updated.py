import bpy
from mathutils import Matrix, Quaternion, Vector

# ============================================================
# CONFIG
# ============================================================

SOURCE_ARMATURE_NAME = "YYB式初音ミクv1.02_arm"
TARGET_ARMATURE_NAME = "Armature"

# Bone mapping: target_bone_name : source_bone_name
BONE_MAP = {
    "pelvis": "下半身",
    "torso": "上半身2",
    "left_upper_arm": "腕.L",
    "left_lower_arm": "ひじ.L",
    "left_hand": "手首.L",
    "right_upper_arm": "腕.R",
    "right_lower_arm": "ひじ.R",
    "right_hand": "手首.R",
    "left_upper_leg": "足.L",
    "left_lower_leg": "ひざ.L",
    "left_foot": "足首.L",
    "right_upper_leg": "足.R",
    "right_lower_leg": "ひざ.R",
    "right_foot": "足首.R",
}

# Rotation source for pelvis comes from BONE_MAP["pelvis"] (e.g. "下半身")
# Translation source for pelvis comes from a separate source bone (e.g. "センター")
TRANSLATION_ROOT_SOURCE = "センター"
TRANSLATION_ROOT_TARGET = "pelvis"

AUTO_MAP_SAME_NAMES = False
IGNORE_TWIST = False
BONE_AXIS_LOCAL = Vector((0.0, 1.0, 0.0))

# New: use precomputed rest orientation offsets so rest frames do NOT need to match
USE_REST_ORIENTATION_OFFSETS = True

USE_SCENE_FRAME_RANGE = True
FRAME_START = 1
FRAME_END = 250

INSERT_KEYFRAMES = True
CLEAR_EXISTING_KEYS = True

# UI responsiveness / throughput
FRAMES_PER_TICK = 1   # increase to 2~5 for speed
TIMER_STEP_SEC = 0.0  # 0.0 = ASAP (still yields to UI event loop)

# ============================================================
# HELPERS
# ============================================================

def quat_normalized_safe(q: Quaternion) -> Quaternion:
    mag2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z
    if mag2 < 1e-16:
        return Quaternion((1.0, 0.0, 0.0, 0.0))
    mag = mag2 ** 0.5
    return Quaternion((q.w / mag, q.x / mag, q.y / mag, q.z / mag))


def rot3_orthonormalized(m3: Matrix) -> Matrix:
    r = m3.copy()
    r.normalize()
    return r


def remove_twist_from_quat(q: Quaternion, twist_axis_world: Vector) -> Quaternion:
    qn = quat_normalized_safe(q)
    axis = twist_axis_world.normalized()
    v = Vector((qn.x, qn.y, qn.z))
    proj = axis * v.dot(axis)
    twist = Quaternion((qn.w, proj.x, proj.y, proj.z))
    twist = quat_normalized_safe(twist)
    swing = qn @ twist.inverted()
    return quat_normalized_safe(swing)


def get_bone_depth(arm_obj, bone_name):
    bone = arm_obj.data.bones.get(bone_name)
    d = 0
    while bone and bone.parent:
        d += 1
        bone = bone.parent
    return d


def get_rest_world_matrix(arm_obj, bone_name) -> Matrix:
    return arm_obj.matrix_world @ arm_obj.data.bones[bone_name].matrix_local


def get_pose_world_matrix(arm_obj, bone_name) -> Matrix:
    return arm_obj.matrix_world @ arm_obj.pose.bones[bone_name].matrix


def build_bone_map(source_obj, target_obj):
    bone_map = dict(BONE_MAP)

    if AUTO_MAP_SAME_NAMES:
        src_names = set(source_obj.pose.bones.keys())
        tgt_names = set(target_obj.pose.bones.keys())
        for name in sorted(src_names & tgt_names):
            if name not in bone_map:
                bone_map[name] = name

    valid = {}
    for tgt_name, src_name in bone_map.items():
        if source_obj.pose.bones.get(src_name) and target_obj.pose.bones.get(tgt_name):
            valid[tgt_name] = src_name
        else:
            print(f"[WARN] Skipping invalid bone map: target '{tgt_name}' <- source '{src_name}'")
    return valid


def get_source_delta_world(source_obj, source_bone_name):
    src_rest_w = get_rest_world_matrix(source_obj, source_bone_name)
    src_pose_w = get_pose_world_matrix(source_obj, source_bone_name)
    return src_pose_w @ src_rest_w.inverted()


def compute_rest_orientation_offsets(source_obj, target_obj, bone_map):
    """
    Per target bone:
      R_offset_world = R_tgt_rest_world * inv(R_src_rest_world)
    so we can do:
      R_tgt_pose_world = R_offset_world * R_src_pose_world
    """
    offsets = {}
    for tgt_name, src_name in bone_map.items():
        src_rest_r = rot3_orthonormalized(get_rest_world_matrix(source_obj, src_name).to_3x3())
        tgt_rest_r = rot3_orthonormalized(get_rest_world_matrix(target_obj, tgt_name).to_3x3())
        offsets[tgt_name] = rot3_orthonormalized(tgt_rest_r @ src_rest_r.inverted())
    return offsets


def get_source_pose_world_rot(source_obj, source_bone_name):
    pose_w = get_pose_world_matrix(source_obj, source_bone_name)
    return rot3_orthonormalized(pose_w.to_3x3())


def get_target_desired_world_rotation(
    source_obj,
    target_obj,
    source_bone_name,
    target_bone_name,
    rest_rot_offsets=None,
):
    """
    Returns desired target WORLD rotation (3x3).
    Supports two modes:
      - offset mode: R_tgt = R_offset * R_src_pose
      - old delta mode: R_tgt = delta_src * R_tgt_rest
    """
    if USE_REST_ORIENTATION_OFFSETS and rest_rot_offsets is not None:
        src_pose_r = get_source_pose_world_rot(source_obj, source_bone_name)

        # Optional twist removal on source motion relative to source rest:
        if IGNORE_TWIST:
            src_rest_r = rot3_orthonormalized(get_rest_world_matrix(source_obj, source_bone_name).to_3x3())
            delta_r = rot3_orthonormalized(src_pose_r @ src_rest_r.inverted())
            twist_axis_w = (src_rest_r @ BONE_AXIS_LOCAL).normalized()
            dq = remove_twist_from_quat(delta_r.to_quaternion(), twist_axis_w)
            delta_r = rot3_orthonormalized(dq.to_matrix())

            # Reconstruct pose rot with twist-removed delta
            src_pose_r = rot3_orthonormalized(delta_r @ src_rest_r)

        tgt_desired_world_r = rot3_orthonormalized(rest_rot_offsets[target_bone_name] @ src_pose_r)
        return tgt_desired_world_r

    # Fallback (old assumption: same rest orientation)
    src_delta_w = get_source_delta_world(source_obj, source_bone_name)
    src_delta_world_r = rot3_orthonormalized(src_delta_w.to_3x3())

    if IGNORE_TWIST:
        src_rest_r = rot3_orthonormalized(get_rest_world_matrix(source_obj, source_bone_name).to_3x3())
        twist_axis_w = (src_rest_r @ BONE_AXIS_LOCAL).normalized()
        dq = remove_twist_from_quat(src_delta_world_r.to_quaternion(), twist_axis_w)
        src_delta_world_r = rot3_orthonormalized(dq.to_matrix())

    tgt_rest_r = rot3_orthonormalized(get_rest_world_matrix(target_obj, target_bone_name).to_3x3())
    return rot3_orthonormalized(src_delta_world_r @ tgt_rest_r)


def build_desired_target_pose_matrix_objspace(
    source_obj,
    target_obj,
    source_bone_name,               # rotation source
    target_bone_name,
    allow_translation=False,
    translation_source_bone_name=None,
    rest_rot_offsets=None,
):
    """
    Builds desired PoseBone.matrix in target ARMATURE OBJECT SPACE.
    - Root: translation + rotation
    - Non-root: rotation only
    """
    # --- Rotation (with rest-offset mapping support)
    tgt_desired_world_r = get_target_desired_world_rotation(
        source_obj=source_obj,
        target_obj=target_obj,
        source_bone_name=source_bone_name,
        target_bone_name=target_bone_name,
        rest_rot_offsets=rest_rot_offsets,
    )

    arm_world_r = rot3_orthonormalized(target_obj.matrix_world.to_3x3())
    tgt_desired_obj_r = rot3_orthonormalized(arm_world_r.inverted() @ tgt_desired_world_r)

    tgt_pb = target_obj.pose.bones[target_bone_name]
    cur_obj_pose = tgt_pb.matrix.copy()

    # --- Translation (separate configurable source for root)
    if allow_translation:
        if translation_source_bone_name is None:
            translation_source_bone_name = source_bone_name

        src_delta_w_for_translation = get_source_delta_world(source_obj, translation_source_bone_name)
        tgt_rest_w = get_rest_world_matrix(target_obj, target_bone_name)

        # Root translation uses full delta from translation source
        tgt_desired_world_full = src_delta_w_for_translation @ tgt_rest_w
        tgt_desired_obj_full = target_obj.matrix_world.inverted() @ tgt_desired_world_full
        loc = tgt_desired_obj_full.to_translation()
    else:
        # preserve current object-space translation while setting matrix;
        # channel location will be forced to zero after assignment
        loc = cur_obj_pose.to_translation()

    return Matrix.LocRotScale(
        loc,
        tgt_desired_obj_r.to_quaternion(),
        Vector((1.0, 1.0, 1.0)),
    )


def clear_existing_keyframes_for_mapped_bones(target_obj, bone_map, translation_root_target):
    ad = target_obj.animation_data
    if not ad or not ad.action:
        return

    action = ad.action
    mapped = set(bone_map.keys())
    mapped.add(translation_root_target)

    to_remove = []
    for fc in action.fcurves:
        dp = fc.data_path
        for b in mapped:
            prefix = f'pose.bones["{b}"].'
            if dp.startswith(prefix) and (
                dp.endswith("location") or
                dp.endswith("rotation_quaternion") or
                dp.endswith("scale")
            ):
                to_remove.append(fc)
                break

    for fc in to_remove:
        action.fcurves.remove(fc)

    if to_remove:
        print(f"[INFO] Removed {len(to_remove)} existing F-curves for mapped/translation-root bones.")


def clear_target_pose_for_frame(target_obj, bone_map):
    for tgt_name in bone_map.keys():
        pb = target_obj.pose.bones[tgt_name]
        pb.rotation_mode = 'QUATERNION'
        pb.rotation_quaternion = Quaternion((1.0, 0.0, 0.0, 0.0))
        pb.scale = Vector((1.0, 1.0, 1.0))
        pb.location = Vector((0.0, 0.0, 0.0))
    bpy.context.view_layer.update()


def retarget_frame(
    source_obj,
    target_obj,
    bone_map,
    translation_root_target,
    translation_root_source,
    rest_rot_offsets,
):
    sorted_target_bones = sorted(bone_map.keys(), key=lambda n: get_bone_depth(target_obj, n))

    clear_target_pose_for_frame(target_obj, bone_map)

    for tgt_name in sorted_target_bones:
        src_name = bone_map[tgt_name]
        pb = target_obj.pose.bones[tgt_name]
        allow_translation = (tgt_name == translation_root_target)

        desired_obj_pose = build_desired_target_pose_matrix_objspace(
            source_obj=source_obj,
            target_obj=target_obj,
            source_bone_name=src_name,  # rotation source from BONE_MAP
            target_bone_name=tgt_name,
            allow_translation=allow_translation,
            translation_source_bone_name=(translation_root_source if allow_translation else None),
            rest_rot_offsets=rest_rot_offsets,
        )

        pb.matrix = desired_obj_pose
        pb.scale = Vector((1.0, 1.0, 1.0))

        # keep non-root connected to parent (no local translation channel)
        if not allow_translation:
            pb.location = Vector((0.0, 0.0, 0.0))

        bpy.context.view_layer.update()


# ============================================================
# MODAL OPERATOR (responsive UI, ESC cancel)
# ============================================================

class WM_OT_modal_retarget_motion(bpy.types.Operator):
    bl_idname = "wm.modal_retarget_motion"
    bl_label = "Modal Retarget Motion"
    bl_description = "Retarget animation without freezing Blender UI (ESC to cancel)"
    bl_options = {'REGISTER'}

    _timer = None
    _state = None

    def _cleanup(self, context):
        wm = context.window_manager
        if self._timer is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None
        try:
            wm.progress_end()
        except Exception:
            pass
        if context.area:
            context.area.tag_redraw()

    def _fail(self, context, msg):
        self.report({'ERROR'}, msg)
        print("[ERROR]", msg)
        self._cleanup(context)
        return {'CANCELLED'}

    def invoke(self, context, event):
        source_obj = bpy.data.objects.get(SOURCE_ARMATURE_NAME)
        target_obj = bpy.data.objects.get(TARGET_ARMATURE_NAME)

        if source_obj is None or target_obj is None:
            return self._fail(context, "Could not find source/target armature objects.")
        if source_obj.type != 'ARMATURE' or target_obj.type != 'ARMATURE':
            return self._fail(context, "Source/target objects must be ARMATUREs.")

        bone_map = build_bone_map(source_obj, target_obj)
        if not bone_map:
            return self._fail(context, "No valid bone mappings found.")

        if TRANSLATION_ROOT_TARGET not in bone_map:
            return self._fail(
                context,
                f'TRANSLATION_ROOT_TARGET "{TRANSLATION_ROOT_TARGET}" must be in BONE_MAP target names.'
            )
        if source_obj.pose.bones.get(TRANSLATION_ROOT_SOURCE) is None:
            return self._fail(
                context,
                f'TRANSLATION_ROOT_SOURCE "{TRANSLATION_ROOT_SOURCE}" not found in source armature.'
            )

        # Precompute rest orientation offsets (this is the key new step)
        rest_rot_offsets = compute_rest_orientation_offsets(source_obj, target_obj, bone_map)

        scene = context.scene
        frame_start = scene.frame_start if USE_SCENE_FRAME_RANGE else FRAME_START
        frame_end = scene.frame_end if USE_SCENE_FRAME_RANGE else FRAME_END

        if CLEAR_EXISTING_KEYS:
            clear_existing_keyframes_for_mapped_bones(target_obj, bone_map, TRANSLATION_ROOT_TARGET)

        self._state = {
            "source_obj": source_obj,
            "target_obj": target_obj,
            "bone_map": bone_map,
            "translation_root_target": TRANSLATION_ROOT_TARGET,
            "translation_root_source": TRANSLATION_ROOT_SOURCE,
            "rest_rot_offsets": rest_rot_offsets,
            "frame_start": frame_start,
            "frame_end": frame_end,
            "frame": frame_start,
        }

        print(f"[INFO] Translation source bone: {TRANSLATION_ROOT_SOURCE}")
        print(f"[INFO] Translation target bone: {TRANSLATION_ROOT_TARGET}")
        print(f"[INFO] Rest orientation offsets: {'ENABLED' if USE_REST_ORIENTATION_OFFSETS else 'DISABLED'}")
        print(f"[INFO] Retargeting frames {frame_start}..{frame_end}")
        print(f"[INFO] Bone count: {len(bone_map)}")
        print("[INFO] Modal retarget started. Press ESC to cancel.")

        wm = context.window_manager
        wm.progress_begin(frame_start, frame_end + 1)
        self._timer = wm.event_timer_add(TIMER_STEP_SEC, window=context.window)
        wm.modal_handler_add(self)

        if context.area:
            context.area.tag_redraw()

        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if self._state is None:
            return self._fail(context, "Internal state missing.")

        if event.type == 'ESC':
            print("[INFO] Retarget cancelled by user.")
            self.report({'WARNING'}, "Retarget cancelled.")
            self._cleanup(context)
            return {'CANCELLED'}

        if event.type != 'TIMER':
            return {'PASS_THROUGH'}

        st = self._state
        scene = context.scene

        try:
            for _ in range(max(1, FRAMES_PER_TICK)):
                f = st["frame"]
                if f > st["frame_end"]:
                    print("[INFO] Retargeting complete.")
                    self.report({'INFO'}, "Retarget complete.")
                    self._cleanup(context)
                    return {'FINISHED'}

                scene.frame_set(f)
                bpy.context.view_layer.update()

                retarget_frame(
                    source_obj=st["source_obj"],
                    target_obj=st["target_obj"],
                    bone_map=st["bone_map"],
                    translation_root_target=st["translation_root_target"],
                    translation_root_source=st["translation_root_source"],
                    rest_rot_offsets=st["rest_rot_offsets"],
                )

                if INSERT_KEYFRAMES:
                    for tgt_name in st["bone_map"].keys():
                        pb = st["target_obj"].pose.bones[tgt_name]
                        pb.keyframe_insert(data_path="rotation_quaternion", frame=f)
                        pb.keyframe_insert(data_path="scale", frame=f)
                        pb.keyframe_insert(data_path="location", frame=f)

                context.window_manager.progress_update(f)

                if f % 10 == 0:
                    print(f"[INFO] frame {f}/{st['frame_end']}")

                st["frame"] += 1

            # redraw all areas
            for window in context.window_manager.windows:
                for area in window.screen.areas:
                    area.tag_redraw()

            return {'RUNNING_MODAL'}

        except Exception as e:
            import traceback
            traceback.print_exc()
            return self._fail(context, f"Retarget error: {e}")


# ============================================================
# REGISTER / RUN
# ============================================================

classes = (
    WM_OT_modal_retarget_motion,
)

def register():
    for c in classes:
        bpy.utils.register_class(c)

def unregister():
    for c in reversed(classes):
        bpy.utils.unregister_class(c)

# Safe re-register for repeated runs in Blender text editor
try:
    unregister()
except Exception:
    pass
register()

# Run operator (responsive UI, ESC cancels)
bpy.ops.wm.modal_retarget_motion('INVOKE_DEFAULT')
