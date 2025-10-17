class DisjointSet:
    
    def __init__(self):
        self.pai = {}
        self.rank = {}

    def make_set(self, x):
        self.pai[x] = x
        self.rank[x] = 0

    def find(self, x):
        if self.pai[x] != x:
            # compressão de caminho
            self.pai[x] = self.find(self.pai[x])
        return self.pai[x]

    def union(self, x, y):
        raiz_x = self.find(x)
        raiz_y = self.find(y)

        if raiz_x == raiz_y:
            return False  # já estão no mesmo conjunto

        # união por rank
        if self.rank[raiz_x] < self.rank[raiz_y]:
            self.pai[raiz_x] = raiz_y
        elif self.rank[raiz_x] > self.rank[raiz_y]:
            self.pai[raiz_y] = raiz_x
        else:
            self.pai[raiz_y] = raiz_x
            self.rank[raiz_x] += 1

        return True
