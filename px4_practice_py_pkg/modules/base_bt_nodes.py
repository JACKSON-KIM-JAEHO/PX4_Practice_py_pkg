from enum import Enum


# =====================================================
# BT Status
# =====================================================

class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


# =====================================================
# Base BT Node
# =====================================================

class BTNode:
    def __init__(self, name):
        self.name = name
        self.status = None

    def tick(self):
        raise NotImplementedError

    def halt(self):
        pass

    def reset(self):
        self.halt()
        self.status = None


# =====================================================
# Control Nodes
# =====================================================

class Sequence(BTNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_index = 0

    def tick(self):
        while self.current_index < len(self.children):
            status = self.children[self.current_index].tick()
            self.status = status

            if status == Status.RUNNING:
                return Status.RUNNING

            if status == Status.FAILURE:
                self._reset_children()
                self.current_index = 0
                return Status.FAILURE

            self.current_index += 1

        self._reset_children()
        self.current_index = 0
        return Status.SUCCESS

    def _reset_children(self):
        for c in self.children:
            c.reset()


class ReactiveSequence(BTNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for c in self.children:
            status = c.tick()
            self.status = status

            if status == Status.FAILURE:
                self._reset_children()
                return Status.FAILURE

            if status == Status.RUNNING:
                return Status.RUNNING

        self._reset_children()
        return Status.SUCCESS

    def _reset_children(self):
        for c in self.children:
            c.reset()


class Fallback(BTNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_index = 0

    def tick(self):
        while self.current_index < len(self.children):
            status = self.children[self.current_index].tick()
            self.status = status

            if status == Status.RUNNING:
                return Status.RUNNING

            if status == Status.SUCCESS:
                self._reset_children()
                self.current_index = 0
                return Status.SUCCESS

            self.current_index += 1

        self._reset_children()
        self.current_index = 0
        return Status.FAILURE

    def _reset_children(self):
        for c in self.children:
            c.reset()


class ReactiveFallback(BTNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for c in self.children:
            status = c.tick()
            self.status = status

            if status == Status.SUCCESS:
                self._reset_children()
                return Status.SUCCESS

            if status == Status.RUNNING:
                return Status.RUNNING

        self._reset_children()
        return Status.FAILURE

    def _reset_children(self):
        for c in self.children:
            c.reset()

class Parallel(BTNode):
    def __init__(self, name, children, success_count=None, failure_count=None):
        super().__init__(name)
        self.children = children
        self.success_count = success_count
        self.failure_count = failure_count

    def tick(self):
        successes = 0
        failures = 0
        running = False

        for c in self.children:
            status = c.tick()

            if status == Status.SUCCESS:
                successes += 1
            elif status == Status.FAILURE:
                failures += 1
            elif status == Status.RUNNING:
                running = True

        # ❌ success_count = -1 → 절대 SUCCESS 안 함
        if self.success_count is not None and self.success_count >= 0:
            if successes >= self.success_count:
                self._reset_children()
                return Status.SUCCESS

        if self.failure_count is not None:
            if failures >= self.failure_count:
                self._reset_children()
                return Status.FAILURE

        # RUNNING이면 reset 금지
        if running:
            return Status.RUNNING

        # 아무 조건도 안 맞으면 RUNNING 유지
        return Status.RUNNING

    def _reset_children(self):
        for c in self.children:
            c.reset()


'''
class Parallel(BTNode):
    def __init__(self, name, children, success_count=None, failure_count=None):
        super().__init__(name)
        self.children = children
        self.success_count = success_count or len(children)
        self.failure_count = failure_count

    def tick(self):
        successes = 0
        failures = 0
        running = False

        for c in self.children:
            status = c.tick()

            if status == Status.SUCCESS:
                successes += 1
            elif status == Status.FAILURE:
                failures += 1
            elif status == Status.RUNNING:
                running = True

        if successes >= self.success_count:
            self._reset_children()
            return Status.SUCCESS

        if self.failure_count is not None and failures >= self.failure_count:
            self._reset_children()
            return Status.FAILURE

        if running:
            return Status.RUNNING

        self._reset_children()
        return Status.FAILURE

    def _reset_children(self):
        for c in self.children:
            c.reset()'''
