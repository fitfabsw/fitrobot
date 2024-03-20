class ListQueue:
    def __init__(self):
        self.queue = []
    def qsize(self):
        return len(self.queue)
    def get(self, idx):
        if self.qsize()> idx >= 0:
            return self.queue[idx]
        else:
            raise Exception(f"佇列取值超出範圍 {self.qsize()}>{idx}>=0 不成立")
    def put(self, item):
        self.queue.append(item)
    def remove_nth(self, n):
        self.queue.pop(n)
    