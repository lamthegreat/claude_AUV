def test_import_package():
    import auv_motor_controller  # noqa: F401


def test_import_modules():
    from auv_motor_controller import motor_controller_node  # noqa: F401
    from auv_motor_controller import thruster_allocator  # noqa: F401
