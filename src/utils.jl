# Miscellaneous utilities

"""
    lazy_collect(collection)
    lazy_collect(element_type, collection)

Lazily `collect` an iterable. Returns the original `collection` if it is already
an `Array` of the appropriate type.
"""
lazy_collect(collection) = collect(collection)
lazy_collect(collection::Array) = collection
lazy_collect(element_type::Type, collection) = collect(element_type, collection)
lazy_collect(::Type{T}, collection::Array{T}) where {T} = collection

"Automatically generate `Base.hash` for a user-defined (composite) type."
macro auto_hash(type)
    expr = quote
        names = fieldnames($(esc(type)))
        eval(AutoHashEquals.auto_hash($(esc(type)), names))
    end
    return expr
end

"Automatically generate `Base.(==)` for a user-defined (composite) type."
macro auto_equals(type)
    expr = quote
        names = fieldnames($(esc(type)))
        eval(AutoHashEquals.auto_equals($(esc(type)), names))
    end
    return expr
end

"Convert vector of unnormalized scores to probabiities."
function softmax(scores)
    if isempty(scores) return Float64[] end
    ws = exp.(scores .- maximum(scores))
    z = sum(ws)
    return isnan(z) ? ones(length(scores)) ./ length(scores) : ws ./ z
end

"Return sample from the standard Gumbel distribution."
function randgumbel(rng::AbstractRNG=Random.GLOBAL_RNG)
    return -log(-log(rand(rng)))
end
