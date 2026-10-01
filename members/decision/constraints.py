"""Private decision constraints; no public interface fields are added."""


class Constraint(object):
    def __init__(self, kind, source, identifier, reason, distance=None,
                 speed=None, valid_until=0.0):
        self.kind = kind
        self.source = source
        self.identifier = identifier
        self.reason = reason
        self.distance = distance
        self.speed = speed
        self.valid_until = valid_until

    def key(self):
        return (self.distance if self.distance is not None else float("inf"),
                self.source, str(self.identifier))


class ConstraintSet(object):
    def __init__(self, speed_limit):
        self.speed_limit = float(speed_limit)
        self.items = []

    def add(self, kind, source, identifier, reason, distance=None,
            speed=None, valid_until=0.0):
        item = Constraint(kind, source, identifier, reason, distance,
                          speed, valid_until)
        self.items.append(item)
        return item

    def first(self, kind):
        items = [item for item in self.items if item.kind == kind]
        return min(items, key=lambda item: item.key()) if items else None

    def stop(self):
        return self.first("STOP")

    def speed(self):
        limits = [item.speed for item in self.items
                  if item.kind == "FOLLOW" and item.speed is not None]
        return min([self.speed_limit] + limits)

    def follows(self):
        return any(item.kind == "FOLLOW" for item in self.items)

    def reason(self, primary):
        others = sorted((item for item in self.items if item is not primary),
                        key=lambda item: (item.kind, item.source,
                                          str(item.identifier)))
        suffix = "; constraints=" + ",".join(
            "{0}:{1}".format(item.kind, item.source) for item in others)
        return primary.reason + suffix if others else primary.reason
