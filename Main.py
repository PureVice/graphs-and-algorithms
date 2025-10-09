
# %%
from Grafo import Grafo

# %% [markdown]
# Montagem de um grafo arbitrário
# 

# %%
g = Grafo(direcionado=True)

# %%
g

# %%
g.inserir_aresta('a', 'b')
g.inserir_aresta('a', 'c')
g.inserir_aresta('b', 'd')
g.inserir_aresta('c', 'b')
g.inserir_aresta('d', 'c')
g.inserir_aresta('e', 'd')
g.inserir_aresta('e', 'f')


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

g.kosaraju()



# %%
