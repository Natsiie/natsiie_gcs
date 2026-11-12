"""纠偏帧门控：只把稳定、可信的视觉结果发给下位机。

大偏差时底盘会走一大步，运动过程中图像有拖影，检测器可能给出稳定的错误
圆心。若这些帧继续作为 0x04/0x05/0x0E 下发，下位机会追着拖影反复重设
目标，表现为左右摇摆。这里统一用 stable + confidence 作为发送门槛。
"""

from __future__ import annotations


def should_publish_trim(result, require_stable=True,
                         minimum_confidence=0.5) -> bool:
    """判断一帧视觉结果是否允许作为纠偏帧下发。

    ``within`` 不能作为门槛：大偏差时 within=0 正是需要移动的时候。
    """
    if not bool(getattr(result, 'detected', False)):
        return False
    if require_stable and not bool(getattr(result, 'stable', False)):
        return False
    confidence = getattr(result, 'confidence', None)
    if confidence is not None and float(confidence) < float(minimum_confidence):
        return False
    return True
