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

"Generate `Base.hash` for a user-defined (composite) type."
macro auto_hash(type)
    expr = quote
        fields = fieldnames($(esc(type)))
        eval(auto_hash_expr($(esc(type)), fields))
    end
    return expr
end

function auto_hash_expr(type, fields)
    function expand(i)
        if i == 0
            :(h)
        else
            :(hash(a.$(fields[i]), $(expand(i-1))))
        end
    end
    return quote
        function Base.hash(a::$(type), h::UInt)
            $(expand(length(fields)))
        end
    end
end

"Generate `Base.(==)` and `Base.isqual` for a user-defined (composite) type."
macro auto_equals(type)
    expr = quote
        names = fieldnames($(esc(type)))
        eval(auto_equals_expr($(esc(type)), names))
    end
    return expr
end

function auto_equals_expr(type, fields)
    function expand(i, f)
        if i == 0
            :true
        else
            :($f(a.$(fields[i]), b.$(fields[i])) && $(expand(i-1, f)))
        end
    end
    return quote
        function Base.:(==)(a::$(type), b::$(type))
            $(expand(length(fields), :(==)))
        end
        function Base.isequal(a::$(type), b::$(type))
            $(expand(length(fields), :isequal))
        end
    end
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
