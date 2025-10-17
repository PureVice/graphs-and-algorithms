
# %%
from Grafo import GrafoPonderado

# %% [markdown]
# Montagem de um grafo arbitrário
# 

# %%
g = GrafoPonderado()

# %%
g

# %%
g.inserir_aresta('a', 'b', 4)
g.inserir_aresta('a', 'h', 8)
g.inserir_aresta('b', 'h', 11)
g.inserir_aresta('b', 'c', 8)
g.inserir_aresta('c', 'i', 2)
g.inserir_aresta('c', 'f', 4)
g.inserir_aresta('c', 'd', 7)
g.inserir_aresta('d', 'e', 9)
g.inserir_aresta('d', 'f', 14)
g.inserir_aresta('e', 'f', 10)
g.inserir_aresta('f', 'g', 2)
g.inserir_aresta('g', 'i', 6)
g.inserir_aresta('g', 'h', 1)
g.inserir_aresta('i', 'h', 7)


# %%
num_arestas = g.get_numero_arestas()
num_vertices = g.get_numero_vertices()

print(f"arestas: {num_arestas} \nvertices: {num_vertices}")

# %%
g

# %%
#g.dfs()

tempos = g.dfs_com_tempo()

# %%

#cfcs = g.kosaraju()
#print(cfcs)



# %%
print(g)
# %%
print(g.kruskal('a'))
print("########################")

# %%
print(g.prim('a'))