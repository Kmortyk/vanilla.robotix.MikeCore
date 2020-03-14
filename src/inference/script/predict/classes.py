def get_class_name(idx, classes):
    for name, _idx in classes.items():
        if _idx == idx:
            return name
