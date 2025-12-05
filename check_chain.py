from ikpy.chain import Chain
from ikpy.link import URDFLink

URDF_PATH = "/root/piper_ws/piper_ros.urdf"

print(f"Loading URDF from {URDF_PATH}")
chain = Chain.from_urdf_file(URDF_PATH)

print("=== Links in chain ===")
for i, link in enumerate(chain.links):
    link_type = type(link).__name__
    # OriginLink 没有 is_joint，URDFLink 有
    is_joint = getattr(link, "is_joint", False)
    print(f"{i:2d}: type={link_type}, name={link.name}, is_joint={is_joint}, bounds={getattr(link, 'bounds', None)}")
