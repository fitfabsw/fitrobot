class ListQueue:
    def __init__(self):
        self.queue = []
    def qsize(self):
        return len(self.queue)
    def get(self):
        if self.qsize() > 0:
            return self.queue.pop(0)
        else:
            raise Exception("Queue is empty")
    def put(self, item):
        self.queue.append(item)
    def remove_nth(self, n):
        self.queue.pop(n)
    